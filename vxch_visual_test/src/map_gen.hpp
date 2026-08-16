#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include "grid_io.hpp"

// Procedural synthetic occupancy grid: a SLAM-map-like layout (outer walls,
// a few rooms with doorways, scattered furniture-like obstacles, and an
// unexplored fringe) so vxch's progressive Haar reconstruction has real
// structure -- flat walls that show up early in coarse bands, and small
// obstacles that only resolve once the finer detail bands arrive.
namespace vxch_test
{

inline SyntheticGrid generate_synthetic_grid(
  std::uint32_t width, std::uint32_t height, float resolution, unsigned seed)
{
  SyntheticGrid grid;
  grid.width = width;
  grid.height = height;
  grid.resolution = resolution;
  grid.data.assign(static_cast<std::size_t>(width) * height, -1);

  std::mt19937 rng(seed);
  const auto idx = [&](int r, int c) {return static_cast<std::size_t>(r) * width + c;};
  const auto in_bounds = [&](int r, int c) {
      return r >= 0 && c >= 0 && r < static_cast<int>(height) && c < static_cast<int>(width);
    };
  const auto set = [&](int r, int c, std::int8_t v) {
      if (in_bounds(r, c)) {grid.data[idx(r, c)] = v;}
    };

  // Organic exploration frontier: an irregular margin (sum of a few sines
  // plus noise) around the border stays unknown, like a robot hasn't swept
  // all the way to the map edge yet.
  std::uniform_real_distribution<double> phase_dist(0.0, 2.0 * M_PI);
  const double phase_a = phase_dist(rng);
  const double phase_b = phase_dist(rng);
  std::uniform_int_distribution<int> base_margin_dist(
    3, std::max(4, static_cast<int>(std::min(width, height)) / 10));
  const int base_margin = base_margin_dist(rng);

  const auto frontier_margin = [&](double t) {
      const double wobble = 3.0 * std::sin(4.0 * t + phase_a) + 2.0 * std::sin(9.0 * t + phase_b);
      return std::max(1, static_cast<int>(base_margin + wobble));
    };

  std::vector<std::uint8_t> explored(static_cast<std::size_t>(width) * height, 0);
  for (int r = 0; r < static_cast<int>(height); ++r) {
    const int left = frontier_margin(static_cast<double>(r) / height);
    const int right = frontier_margin(1.0 + static_cast<double>(r) / height);
    for (int c = std::max(0, left); c < static_cast<int>(width) - right; ++c) {
      explored[idx(r, c)] = 1;
    }
  }
  for (int c = 0; c < static_cast<int>(width); ++c) {
    const int top = frontier_margin(2.0 + static_cast<double>(c) / width);
    const int bottom = frontier_margin(3.0 + static_cast<double>(c) / width);
    for (int r = 0; r < top; ++r) {explored[idx(r, c)] = 0;}
    for (int r = static_cast<int>(height) - bottom; r < static_cast<int>(height); ++r) {
      if (r >= 0) {explored[idx(r, c)] = 0;}
    }
  }

  int min_r = static_cast<int>(height), max_r = -1, min_c = static_cast<int>(width), max_c = -1;
  for (int r = 0; r < static_cast<int>(height); ++r) {
    for (int c = 0; c < static_cast<int>(width); ++c) {
      if (explored[idx(r, c)]) {
        grid.data[idx(r, c)] = 0;  // free
        min_r = std::min(min_r, r); max_r = std::max(max_r, r);
        min_c = std::min(min_c, c); max_c = std::max(max_c, c);
      }
    }
  }
  if (max_r < min_r || max_c < min_c) {
    // Degenerate (tiny) grid -- leave everything unknown rather than divide by zero below.
    return grid;
  }

  const auto is_explored = [&](int r, int c) {
      return in_bounds(r, c) && explored[idx(r, c)] != 0;
    };
  const auto draw_wall_cell = [&](int r, int c) {
      if (is_explored(r, c)) {set(r, c, 100);}
    };

  // Outer wall ring just inside the explored/unknown boundary.
  for (int c = min_c; c <= max_c; ++c) {
    draw_wall_cell(min_r, c);
    draw_wall_cell(max_r, c);
  }
  for (int r = min_r; r <= max_r; ++r) {
    draw_wall_cell(r, min_c);
    draw_wall_cell(r, max_c);
  }

  // Partition into rooms with a couple of internal walls, each with a
  // doorway gap so rooms are visually distinct but not sealed off.
  const int span_r = max_r - min_r;
  const int span_c = max_c - min_c;
  std::uniform_int_distribution<int> door_width_dist(2, 4);

  if (span_c > 20) {
    const int wall_c = min_c + span_c / 2 + std::uniform_int_distribution<int>(-3, 3)(rng);
    const int door_w = door_width_dist(rng);
    const int door_r = min_r + std::uniform_int_distribution<int>(0, std::max(1, span_r))(rng);
    for (int r = min_r; r <= max_r; ++r) {
      if (r >= door_r && r < door_r + door_w) {continue;}
      draw_wall_cell(r, wall_c);
    }
  }
  if (span_r > 20) {
    const int wall_r = min_r + span_r / 2 + std::uniform_int_distribution<int>(-3, 3)(rng);
    const int door_w = door_width_dist(rng);
    const int door_c = min_c + std::uniform_int_distribution<int>(0, std::max(1, span_c))(rng);
    for (int c = min_c; c <= max_c; ++c) {
      if (c >= door_c && c < door_c + door_w) {continue;}
      draw_wall_cell(wall_r, c);
    }
  }

  // Scattered furniture-like obstacles: small filled rectangles dropped at
  // random free cells.
  const int area = std::max(1, (span_r + 1) * (span_c + 1));
  const int num_obstacles = std::clamp(area / 400, 3, 40);
  std::uniform_int_distribution<int> r_dist(min_r + 2, std::max(min_r + 2, max_r - 2));
  std::uniform_int_distribution<int> c_dist(min_c + 2, std::max(min_c + 2, max_c - 2));
  std::uniform_int_distribution<int> size_dist(1, 3);
  for (int i = 0; i < num_obstacles; ++i) {
    const int cr = r_dist(rng);
    const int cc = c_dist(rng);
    const int hr = size_dist(rng);
    const int hc = size_dist(rng);
    for (int r = cr - hr; r <= cr + hr; ++r) {
      for (int c = cc - hc; c <= cc + hc; ++c) {
        if (is_explored(r, c)) {set(r, c, 100);}
      }
    }
  }

  return grid;
}

}  // namespace vxch_test
