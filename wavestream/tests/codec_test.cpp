#include <wavestream/codec.hpp>
#include <iostream>
#include <random>

std::vector<std::int8_t> decode_from_second_translation_unit(const wavestream::EncodedTile &);
void check(bool ok) {if (!ok) {throw std::runtime_error("codec test failed");}}
template<class F> void rejects(F action)
{
  bool rejected = false;
  try {action();} catch (const std::exception &) {rejected = true;}
  check(rejected);
}
int main()
{
  // Hand-calculated 2x2 example: phi(u) = [0,2;4,6], LL=3,
  // HL=2, LH=4, HH=0. This checks signs, orientation and lifting rounding.
  const std::vector<std::int8_t> example{-1,1,3,5};
  const auto encoded = wavestream::encode_tile(example, 2, 2, 1);
  check(wavestream::decode_band(encoded.bands[0]) == wavestream::Coefficients{3});
  check(wavestream::decode_band(encoded.bands[1]) == (wavestream::Coefficients{2,4,0}));
  check(wavestream::decode_tile(encoded, 1) == std::vector<std::int8_t>(4,2));
  check(decode_from_second_translation_unit(encoded) == example);

  // Diverse shapes include odd boundaries, 1D tiles, and levels deeper than
  // the tile size (valid empty detail groups once an axis reaches one cell).
  std::mt19937 rng(12345);
  for (std::size_t w : {1,2,3,7,16,40}) {
    for (std::size_t h : {1,2,5,8,17}) {
      for (int levels : {1,2,4,7,12}) {
        std::vector<std::int8_t> input(w*h);
        for (auto & v : input) {v = static_cast<std::int8_t>(rng()%102-1);}
        const auto e = wavestream::encode_tile(input,w,h,levels);
        check(e.bands.size() == static_cast<std::size_t>(levels+1));
        check(wavestream::decode_tile(e,e.bands.size()) == input);
        for (std::size_t k=1; k<=e.bands.size(); ++k) {
          const auto partial = wavestream::decode_tile(e,k);
          check(partial.size() == input.size());
          for (const auto v : partial) {check(v >= -1 && v <= 100);}
        }
      }
    }
  }
  for (std::int8_t value : {-1,0,100}) {
    const auto e = wavestream::encode_tile(std::vector<std::int8_t>(35,value),7,5,4);
    for (std::size_t k=1; k<=e.bands.size(); ++k) {
      check(wavestream::decode_tile(e,k) == std::vector<std::int8_t>(35,value));
    }
  }
  const wavestream::Coefficients extremes{0,-1,1,INT64_MIN,INT64_MAX};
  check(wavestream::zigzag_varint_decode(wavestream::zigzag_varint_encode(extremes),5)==extremes);
  rejects([] {wavestream::zigzag_varint_decode({0x80},1);});
  rejects([] {wavestream::zigzag_varint_decode({},1);});
  rejects([] {wavestream::zigzag_varint_decode(wavestream::Bytes(10,0xFF),1);});
  rejects([] {wavestream::fixed_width_decode({},1);});
  rejects([] {wavestream::encode_tile({101},1,1,1);});
  rejects([] {wavestream::encode_tile({0},2,2,1);});
  rejects([] {wavestream::encode_tile({0},1,1,0);});
  rejects([&] {wavestream::decode_tile(encoded,0);});
  rejects([&] {wavestream::decode_tile(encoded,3);});
  auto corrupt = encoded;
  corrupt.bands[0].payload = {0};
  rejects([&] {wavestream::decode_tile(corrupt,1);});
  corrupt = encoded;
  ++corrupt.bands[0].coefficient_count;
  rejects([&] {wavestream::decode_tile(corrupt,1);});
  std::cout << "Standalone codec: round trips, prefixes, malformed input, and multi-TU link passed\n";
}
