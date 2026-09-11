#include <wavestream/codec.hpp>
#include <voxelcodec_ros/haar_forward.hpp>
#include <voxelcodec_ros/tile_scheduler.hpp>
#include <voxelcodec_ros/tile_reconstructor.hpp>
#include <iostream>

void check(bool ok) {if (!ok) {throw std::runtime_error("adapter test failed");}}
int main()
{
  for (std::size_t w : {1,3,8,17}) {
    const std::size_t h=5;
    std::vector<std::int8_t> input(w*h);
    std::vector<std::uint32_t> embedded(w*h);
    for (std::size_t i=0; i<input.size(); ++i) {
      input[i]=static_cast<std::int8_t>((i*31)%102)-1;
      embedded[i]=wavestream::occupancy_to_embedded(input[i]);
    }
    const auto standalone=wavestream::encode_tile(input,w,h,4);
    const auto adapter=voxelcodec_ros::make_haar_bands(embedded,w,h,4,"zstd");
    for (std::size_t j=0;j<adapter.size();++j) {
      check(adapter[j].payload==standalone.bands[j].payload);
      check(adapter[j].descriptor.uncompressed_size==standalone.bands[j].uncompressed_size);
      check(adapter[j].descriptor.element_count==standalone.bands[j].coefficient_count);
    }
    // Exercise real tile/cache adapters, including clipped edge tiles and
    // negative world coordinates, for every existing packing/compression mode.
    for (bool varint : {false,true}) {
      for (const std::string compression : {"none","zstd"}) {
        voxelcodec_ros::TileScheduler sender(0.2,4,compression,varint,"smart");
        const auto ingested=sender.ingest_grid(input,static_cast<int>(w),5,0.05,-0.15,0.05);
        check(ingested.tile_errors.empty());
        voxelcodec_ros::TileReconstructor receiver(4);
        voxelcodec_ros::Metadata geometry{
          {"grid_width",std::to_string(w)},{"grid_height","5"},
          {"tile_size_cells","4"},{"resolution","0.05"},
          {"origin_x","-0.15"},{"origin_y","0.05"}};
        check(receiver.ingest_manifest(geometry,{}));
        for (auto & band : sender.take_pending_bands(5,-1)) {
          check(!receiver.ingest_band(band.band_index,band.channel.descriptor,band.channel.payload));
        }
        const auto result=receiver.reconstruct();
        check(result && result->data==input);
      }
    }
  }
  std::cout << "ROS-free adapters: byte parity and tiled reconstruction passed\n";
}
