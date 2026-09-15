#pragma once

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>
#include <vector>

// Evaluation-only lattice in Gazebo world coordinates. Never fed to navigation.
class ObservedCells {
 public:
  ObservedCells(double resolution, unsigned robots)
      : resolution_(resolution), own(robots, 0) {
    if (!std::isfinite(resolution) || resolution <= 0 || robots == 0 || robots > 64)
      throw std::invalid_argument("positive resolution and 1..64 robots required");
  }

  void ray(unsigned robot, double x0, double y0, double x1, double y1) {
    if (robot >= own.size() || !std::isfinite(x0) || !std::isfinite(y0) ||
        !std::isfinite(x1) || !std::isfinite(y1)) return;
    int x = std::floor(x0 / resolution_), y = std::floor(y0 / resolution_);
    const int ex = std::floor(x1 / resolution_), ey = std::floor(y1 / resolution_);
    const double dx = x1 - x0, dy = y1 - y0;
    const int sx = (dx > 0) - (dx < 0), sy = (dy > 0) - (dy < 0);
    const double inf = INFINITY;
    const double tx = sx ? resolution_ / std::abs(dx) : inf;
    const double ty = sy ? resolution_ / std::abs(dy) : inf;
    double nx = sx ? ((x + (sx > 0)) * resolution_ - x0) / dx : inf;
    double ny = sy ? ((y + (sy > 0)) * resolution_ - y0) / dy : inf;
    // Exclude the terminal cell: it may contain the obstacle return. For a
    // no-return beam this conservatively excludes the range-limit cell too.
    while (x != ex || y != ey) {
      observe(robot, x, y);
      if (nx < ny) { x += sx; nx += tx; }
      else if (ny < nx) { y += sy; ny += ty; }
      else { x += sx; y += sy; nx += tx; ny += ty; }
    }
  }

  std::uint64_t union_count() const { return owners_.size(); }
  std::uint64_t overlap = 0;
  double resolution_;
  std::vector<std::uint64_t> own;

 private:
  std::unordered_map<std::uint64_t, std::uint64_t> owners_;
  void observe(unsigned robot, int x, int y) {
    const auto key = (std::uint64_t(std::uint32_t(x)) << 32) | std::uint32_t(y);
    const auto bit = std::uint64_t{1} << robot;
    auto &mask = owners_[key];
    if (mask & bit) return;
    if (mask && !(mask & (mask - 1))) ++overlap;
    mask |= bit;
    ++own[robot];
  }
};
