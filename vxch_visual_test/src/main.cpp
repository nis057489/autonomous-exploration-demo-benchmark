// Standalone (no ROS) CLI that drives the real vxch codec
// (voxelcodec_ros/codec.cpp + haar_forward.hpp, compiled straight from
// exploration_packages/vxch -- see CMakeLists.txt) against a synthetic
// occupancy grid, so the encode/progressive-decode pipeline can be exercised
// and watched update visually without rclcpp, DDS discovery, or a robot.
//
// Subcommands:
//   gen-map  --out <grid>                      synthesize an occupancy grid
//   encode   --map <grid> --out <session>       build the per-tile Haar band
//                                                send queue (mirrors
//                                                occupancy_grid_vxch_node's
//                                                round-robin order for a
//                                                single static map -- see
//                                                build_send_order below)
//   step     --session <session> --state <state> --out-receiver <grid>
//                                                pop the next N queued bands,
//                                                decode them into the
//                                                receiver's per-tile state
//                                                (mirrors
//                                                vxch_occupancy_grid_node),
//                                                and write out the receiver's
//                                                current reconstruction
//
// gen-map/encode are one-shot; step is meant to be invoked once per "send"
// button click (or a few times per click for "send all"), each time
// re-reading/re-writing --state so the caller (gui/vxch_gui.py) doesn't have
// to keep a long-lived process around.
#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "voxelcodec_ros/codec.hpp"
#include "voxelcodec_ros/haar_forward.hpp"
#include "voxelcodec_ros/types.hpp"

#include "grid_io.hpp"
#include "map_gen.hpp"

using json = nlohmann::json;
using namespace voxelcodec_ros;
using vxch_test::SyntheticGrid;

namespace
{

// Same int8 <-> uint32 shift occupancy_grid_vxch_node.cpp / vxch_occupancy_grid_node.cpp
// use, so the Haar pyramid never has to represent a negative "unknown" value.
std::uint32_t shift_to_uint32(std::int8_t v)
{
  return static_cast<std::uint32_t>(static_cast<int>(v) + 1);
}

std::int8_t unshift_from_uint32(std::uint32_t v)
{
  const int shifted = static_cast<int>(v) - 1;
  return static_cast<std::int8_t>(std::max(-1, std::min(100, shifted)));
}

// Mirrors zigzag_varint_encode in voxelcodec_ros/haar_forward.hpp (the
// decode side lives in vxch_occupancy_grid_node.cpp, not the shared codec
// library, since it's only ever needed by a band consumer).
std::vector<std::int64_t> zigzag_varint_decode(
  const std::vector<std::uint8_t> & raw, std::size_t count)
{
  std::vector<std::int64_t> out;
  out.reserve(count);
  std::size_t offset = 0;
  while (out.size() < count) {
    std::uint64_t value = 0;
    int shift = 0;
    while (offset < raw.size()) {
      const std::uint8_t byte = raw[offset++];
      value |= static_cast<std::uint64_t>(byte & 0x7FU) << shift;
      if ((byte & 0x80U) == 0) {break;}
      shift += 7;
      if (shift >= 64) {throw std::runtime_error("varint overflow");}
    }
    const std::int64_t decoded = (value & 1U)
      ? -static_cast<std::int64_t>((value >> 1U) + 1U)
      : static_cast<std::int64_t>(value >> 1U);
    out.push_back(decoded);
  }
  return out;
}

struct ArgMap
{
  std::map<std::string, std::string> values;

  std::string get(const std::string & key, const std::string & def) const
  {
    const auto it = values.find(key);
    return it == values.end() ? def : it->second;
  }
  int get_int(const std::string & key, int def) const
  {
    const auto it = values.find(key);
    return it == values.end() ? def : std::stoi(it->second);
  }
  double get_double(const std::string & key, double def) const
  {
    const auto it = values.find(key);
    return it == values.end() ? def : std::stod(it->second);
  }
  bool require(const std::string & key) const
  {
    return values.count(key) > 0;
  }
};

ArgMap parse_args(int argc, char ** argv, int start)
{
  ArgMap args;
  for (int i = start; i < argc; ++i) {
    std::string a = argv[i];
    if (a.rfind("--", 0) != 0) {continue;}
    std::string key = a.substr(2);
    std::string value = (i + 1 < argc) ? argv[i + 1] : "";
    args.values[key] = value;
    ++i;
  }
  return args;
}

std::string read_file(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {throw std::runtime_error("could not open " + path);}
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

void write_file(const std::string & path, const std::string & content)
{
  std::ofstream out(path);
  if (!out) {throw std::runtime_error("could not open " + path + " for writing");}
  out << content;
}

// -------------------- gen-map --------------------

int cmd_gen_map(const ArgMap & args)
{
  if (!args.require("out")) {
    std::cerr << "gen-map requires --out <path>\n";
    return 1;
  }
  const std::uint32_t width = static_cast<std::uint32_t>(args.get_int("width", 160));
  const std::uint32_t height = static_cast<std::uint32_t>(args.get_int("height", 120));
  const float resolution = static_cast<float>(args.get_double("resolution", 0.05));
  const unsigned seed = static_cast<unsigned>(args.get_int("seed", 42));

  const SyntheticGrid grid = vxch_test::generate_synthetic_grid(width, height, resolution, seed);
  vxch_test::write_grid(args.get("out", ""), grid);

  std::size_t free_c = 0, occ_c = 0, unk_c = 0;
  for (const auto v : grid.data) {
    if (v < 0) {++unk_c;} else if (v == 0) {++free_c;} else {++occ_c;}
  }
  json result;
  result["width"] = width;
  result["height"] = height;
  result["free_cells"] = free_c;
  result["occupied_cells"] = occ_c;
  result["unknown_cells"] = unk_c;
  std::cout << result.dump() << std::endl;
  return 0;
}

// -------------------- encode --------------------

struct QueueEntry
{
  int tile_row;
  int tile_col;
  int tile_width;
  int tile_height;
  int band_index;
  ChannelDescriptor descriptor;
  std::vector<std::uint8_t> payload;
};

// Build the band send order for a single static map snapshot. The real
// occupancy_grid_vxch_node round-robins tiles with its defaults
// (max_bands_per_update=1, max_tiles_per_update=-1): every tick, each tile
// still holding pending bands sends exactly its next (lowest-index,
// coarsest-first) band, then goes to the back of the queue if it still has
// more. Since nothing here re-senses the map, that collapses to: wave 0 =
// every tile's band_0, wave 1 = every tile's band_1, ... which is what this
// produces directly, without needing the node's recency bookkeeping (that
// machinery exists for handling a map that keeps changing while sending,
// which a single generated snapshot never does).
std::vector<QueueEntry> build_send_order(
  const std::vector<std::vector<EncodedChannel>> & tile_bands,
  const std::vector<std::pair<int, int>> & tile_coords,
  const std::vector<std::pair<int, int>> & tile_dims,
  int total_bands)
{
  std::vector<QueueEntry> order;
  for (int k = 0; k < total_bands; ++k) {
    for (std::size_t t = 0; t < tile_bands.size(); ++t) {
      if (static_cast<std::size_t>(k) >= tile_bands[t].size()) {continue;}
      const auto & band = tile_bands[t][static_cast<std::size_t>(k)];
      QueueEntry e;
      e.tile_row = tile_coords[t].first;
      e.tile_col = tile_coords[t].second;
      e.tile_width = tile_dims[t].first;
      e.tile_height = tile_dims[t].second;
      e.band_index = k;
      e.descriptor = band.descriptor;
      e.payload = band.payload;
      order.push_back(std::move(e));
    }
  }
  return order;
}

int cmd_encode(const ArgMap & args)
{
  if (!args.require("map") || !args.require("out")) {
    std::cerr << "encode requires --map <grid> --out <session>\n";
    return 1;
  }
  const int levels = args.get_int("levels", 4);
  const int tile_size_cells = args.get_int("tile-size-cells", 32);
  const std::string compression = args.get("compression", kCompressionZstd);
  if (tile_size_cells <= 0) {throw std::runtime_error("--tile-size-cells must be > 0");}

  const SyntheticGrid grid = vxch_test::read_grid(args.get("map", ""));
  const int grid_w = static_cast<int>(grid.width);
  const int grid_h = static_cast<int>(grid.height);
  const int tiles_x = (grid_w + tile_size_cells - 1) / tile_size_cells;
  const int tiles_y = (grid_h + tile_size_cells - 1) / tile_size_cells;

  std::vector<std::vector<EncodedChannel>> tile_bands;
  std::vector<std::pair<int, int>> tile_coords;
  std::vector<std::pair<int, int>> tile_dims;

  for (int trow = 0; trow < tiles_y; ++trow) {
    for (int tcol = 0; tcol < tiles_x; ++tcol) {
      const int row0 = trow * tile_size_cells;
      const int col0 = tcol * tile_size_cells;
      const int tw = std::min(tile_size_cells, grid_w - col0);
      const int th = std::min(tile_size_cells, grid_h - row0);
      if (tw <= 0 || th <= 0) {continue;}

      std::vector<std::uint32_t> values(static_cast<std::size_t>(tw) * th);
      for (int r = 0; r < th; ++r) {
        for (int c = 0; c < tw; ++c) {
          const std::size_t src = static_cast<std::size_t>(row0 + r) * grid_w + (col0 + c);
          values[static_cast<std::size_t>(r) * tw + c] = shift_to_uint32(grid.data[src]);
        }
      }

      auto bands = make_haar_bands(
        values, static_cast<std::size_t>(tw), static_cast<std::size_t>(th), levels, compression);
      tile_bands.push_back(std::move(bands));
      tile_coords.emplace_back(trow, tcol);
      tile_dims.emplace_back(tw, th);
    }
  }

  const int total_bands = levels + 1;
  const auto order = build_send_order(tile_bands, tile_coords, tile_dims, total_bands);

  json session;
  session["width"] = grid.width;
  session["height"] = grid.height;
  session["resolution"] = grid.resolution;
  session["levels"] = levels;
  session["tile_size_cells"] = tile_size_cells;
  session["tiles_x"] = tiles_x;
  session["tiles_y"] = tiles_y;
  session["raw_bytes"] = static_cast<std::uint64_t>(grid.width) * grid.height;

  std::uint64_t total_compressed = 0;
  json entries = json::array();
  for (const auto & e : order) {
    json je;
    je["tile_row"] = e.tile_row;
    je["tile_col"] = e.tile_col;
    je["tile_width"] = e.tile_width;
    je["tile_height"] = e.tile_height;
    je["band_index"] = e.band_index;
    je["compression"] = e.descriptor.compression;
    je["element_count"] = e.descriptor.element_count;
    je["uncompressed_size"] = e.descriptor.uncompressed_size;
    je["compressed_size"] = e.descriptor.compressed_size;
    je["payload"] = e.payload;
    entries.push_back(std::move(je));
    total_compressed += e.descriptor.compressed_size;
  }
  session["total_compressed_bytes"] = total_compressed;
  session["entries"] = std::move(entries);

  write_file(args.get("out", ""), session.dump());

  json result;
  result["tiles"] = tile_bands.size();
  result["bands_per_tile"] = total_bands;
  result["total_entries"] = order.size();
  result["raw_bytes"] = session["raw_bytes"];
  result["total_compressed_bytes"] = total_compressed;
  std::cout << result.dump() << std::endl;
  return 0;
}

// -------------------- step --------------------

struct TileState
{
  int width{0};
  int height{0};
  int levels{0};
  int total_bands{0};
  std::vector<std::vector<std::int64_t>> band_coeffs;
  std::vector<bool> received;
};

int cmd_step(const ArgMap & args)
{
  if (!args.require("session") || !args.require("state") || !args.require("out-receiver")) {
    std::cerr << "step requires --session <session> --state <state> --out-receiver <grid>\n";
    return 1;
  }
  const int n = args.get_int("n", 1);

  const json session = json::parse(read_file(args.get("session", "")));
  const auto & entries = session.at("entries");
  const std::uint32_t grid_width = session.at("width").get<std::uint32_t>();
  const std::uint32_t grid_height = session.at("height").get<std::uint32_t>();
  const int tile_size_cells = session.at("tile_size_cells").get<int>();

  json state;
  const std::string state_path = args.get("state", "");
  {
    std::ifstream probe(state_path);
    if (probe) {
      state = json::parse(read_file(state_path));
    } else {
      state["sent_index"] = 0;
      state["cumulative_bytes"] = 0;
      state["tiles"] = json::object();
    }
  }

  std::size_t sent_index = state.at("sent_index").get<std::size_t>();
  std::uint64_t cumulative_bytes = state.at("cumulative_bytes").get<std::uint64_t>();

  json sent_this_call = json::array();
  for (int i = 0; i < n && sent_index < entries.size(); ++i) {
    const auto & entry = entries[sent_index];
    ++sent_index;

    ChannelDescriptor desc;
    desc.compression = entry.at("compression").get<std::string>();
    desc.uncompressed_size = entry.at("uncompressed_size").get<std::uint64_t>();
    const auto payload = entry.at("payload").get<std::vector<std::uint8_t>>();
    const auto element_count = entry.at("element_count").get<std::size_t>();

    const auto raw = decompress_payload(desc, payload);
    const auto coeffs = zigzag_varint_decode(raw, element_count);

    const int trow = entry.at("tile_row").get<int>();
    const int tcol = entry.at("tile_col").get<int>();
    const int band_index = entry.at("band_index").get<int>();
    const std::string key = std::to_string(trow) + "_" + std::to_string(tcol);

    if (!state["tiles"].contains(key)) {
      json jt;
      jt["width"] = entry.at("tile_width").get<int>();
      jt["height"] = entry.at("tile_height").get<int>();
      jt["levels"] = session.at("levels").get<int>();
      const int total_bands = jt["levels"].get<int>() + 1;
      jt["total_bands"] = total_bands;
      jt["band_coeffs"] = json::array();
      jt["received"] = json::array();
      for (int b = 0; b < total_bands; ++b) {
        jt["band_coeffs"].push_back(json::array());
        jt["received"].push_back(false);
      }
      state["tiles"][key] = std::move(jt);
    }
    state["tiles"][key]["band_coeffs"][static_cast<std::size_t>(band_index)] = coeffs;
    state["tiles"][key]["received"][static_cast<std::size_t>(band_index)] = true;

    const auto compressed_size = entry.at("compressed_size").get<std::uint64_t>();
    cumulative_bytes += compressed_size;

    json js;
    js["tile_row"] = trow;
    js["tile_col"] = tcol;
    js["band_index"] = band_index;
    js["compressed_size"] = compressed_size;
    js["uncompressed_size"] = entry.at("uncompressed_size");
    js["element_count"] = element_count;
    sent_this_call.push_back(std::move(js));
  }

  state["sent_index"] = sent_index;
  state["cumulative_bytes"] = cumulative_bytes;

  // Re-render the full receiver grid from scratch every step -- simplest
  // correct thing (tile count/size here is small enough it's effectively
  // instant), and it exactly matches vxch_occupancy_grid_node's own
  // publish_map(): unknown everywhere, then paste each tile's current
  // best reconstruction (nearest-neighbour upsampled if not all bands for
  // that tile have arrived yet) over it.
  std::vector<std::int8_t> receiver_data(
    static_cast<std::size_t>(grid_width) * grid_height, -1);

  for (auto it = state["tiles"].begin(); it != state["tiles"].end(); ++it) {
    const json & jt = it.value();
    const int tw = jt.at("width").get<int>();
    const int th = jt.at("height").get<int>();
    const int levels = jt.at("levels").get<int>();
    const int total_bands = jt.at("total_bands").get<int>();
    const auto & received = jt.at("received");

    int bands_received = 0;
    for (int k = 0; k < total_bands; ++k) {
      if (received[static_cast<std::size_t>(k)].get<bool>()) {
        bands_received = k + 1;
      } else {
        break;
      }
    }
    if (bands_received == 0) {continue;}

    std::vector<std::vector<std::int64_t>> band_coeffs(static_cast<std::size_t>(total_bands));
    const auto & jbc = jt.at("band_coeffs");
    for (int k = 0; k < bands_received; ++k) {
      band_coeffs[static_cast<std::size_t>(k)] =
        jbc[static_cast<std::size_t>(k)].get<std::vector<std::int64_t>>();
    }

    const auto recon = reconstruct_haar_from_bands(
      band_coeffs, static_cast<std::size_t>(tw), static_cast<std::size_t>(th),
      levels, bands_received);

    const std::string key = it.key();
    const std::size_t sep = key.find('_');
    const int trow = std::stoi(key.substr(0, sep));
    const int tcol = std::stoi(key.substr(sep + 1));
    const std::size_t row0 = static_cast<std::size_t>(trow) * tile_size_cells;
    const std::size_t col0 = static_cast<std::size_t>(tcol) * tile_size_cells;
    const std::size_t w_prime = recon.width;
    const std::size_t h_prime = recon.height;

    for (int r = 0; r < th; ++r) {
      const std::size_t r_src = (w_prime == static_cast<std::size_t>(tw) &&
        h_prime == static_cast<std::size_t>(th)) ?
        static_cast<std::size_t>(r) : (static_cast<std::size_t>(r) * h_prime / th);
      const std::size_t dst_row = row0 + static_cast<std::size_t>(r);
      if (dst_row >= grid_height) {break;}
      for (int c = 0; c < tw; ++c) {
        const std::size_t c_src = (w_prime == static_cast<std::size_t>(tw) &&
          h_prime == static_cast<std::size_t>(th)) ?
          static_cast<std::size_t>(c) : (static_cast<std::size_t>(c) * w_prime / tw);
        const std::size_t dst_col = col0 + static_cast<std::size_t>(c);
        if (dst_col >= grid_width) {continue;}
        receiver_data[dst_row * grid_width + dst_col] =
          unshift_from_uint32(recon.values[r_src * w_prime + c_src]);
      }
    }
  }

  SyntheticGrid receiver_grid;
  receiver_grid.width = grid_width;
  receiver_grid.height = grid_height;
  receiver_grid.resolution = session.at("resolution").get<float>();
  receiver_grid.data = std::move(receiver_data);
  vxch_test::write_grid(args.get("out-receiver", ""), receiver_grid);

  write_file(state_path, state.dump());

  json result;
  result["sent"] = std::move(sent_this_call);
  result["sent_index"] = sent_index;
  result["total_entries"] = entries.size();
  result["cumulative_bytes"] = cumulative_bytes;
  result["total_compressed_bytes"] = session.at("total_compressed_bytes");
  result["raw_bytes"] = session.at("raw_bytes");
  result["done"] = sent_index >= entries.size();
  std::cout << result.dump() << std::endl;
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "usage: vxch_cli <gen-map|encode|step> [--key value ...]\n";
    return 1;
  }
  const std::string cmd = argv[1];
  const ArgMap args = parse_args(argc, argv, 2);

  try {
    if (cmd == "gen-map") {return cmd_gen_map(args);}
    if (cmd == "encode") {return cmd_encode(args);}
    if (cmd == "step") {return cmd_step(args);}
    std::cerr << "unknown subcommand: " << cmd << "\n";
    return 1;
  } catch (const std::exception & e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
}
