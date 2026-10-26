#include "refresh_icons.h"

#include <array>
#include <cstdint>

namespace refresh_icons {
namespace {

constexpr size_t kFrameBytes = kSize * kSize * sizeof(uint16_t);

// Each string is one 16-pixel row; # is black and . is white.
constexpr char kFrameRows[kFrameCount][kSize][kSize + 1] = {
    {
        "................",
        "................",
        ".......##.......",
        ".......##.......",
        ".......##......",
        "......#..#......",
        "......#..#......",
        ".......##.......",
        "................",
        ".......##.......",
        "....########....",
        "....#......#....",
        "....#......#....",
        ".....#....#.....",
        "......####......",
        "................",
    },
    {
        "................",
        "................",
        "................",
        "................",
        "................",
        "................",
        "......##........",
        ".....#..#.......",
        ".......##.......",
        ".......##.......",
        "....########....",
        "....#......#....",
        "....#......#....",
        ".....#....#.....",
        "......####......",
        "................",
    },
    {
        "................",
        "................",
        "................",
        "....###.##......",
        "...#...#..#.....",
        ".......#.......",
        ".......#.##....",
        ".......##..#....",
        ".......##.......",
        ".......##.......",
        "....########....",
        "....#......#....",
        "....#......#....",
        ".....#....#.....",
        "......####......",
        "................",
    },
    {
        ".....###.###....",
        "....#...#...#...",
        ".......##.......",
        "....##.##.......",
        "...#..###......",
        ".......##.###...",
        "...###.###...#..",
        "..#...###.......",
        ".......##.......",
        ".......##.......",
        "....########....",
        "....#......#....",
        "....#......#....",
        ".....#....#.....",
        "......####......",
        "................",
    },
};

constexpr std::array<std::array<uint8_t, kFrameBytes>, kFrameCount> make_frame_data()
{
    std::array<std::array<uint8_t, kFrameBytes>, kFrameCount> frames{};
    for (size_t frame_index = 0; frame_index < kFrameCount; ++frame_index) {
        for (size_t y = 0; y < kSize; ++y) {
            for (size_t x = 0; x < kSize; ++x) {
                const uint8_t channel =
                    kFrameRows[frame_index][y][x] == '#' ? 0x00 : 0xFF;
                const size_t offset = (y * kSize + x) * sizeof(uint16_t);
                frames[frame_index][offset] = channel;
                frames[frame_index][offset + 1] = channel;
            }
        }
    }
    return frames;
}

constexpr auto kFrameData = make_frame_data();
std::array<lv_image_dsc_t, kFrameCount> s_descriptors{};

}  // namespace

void initialize()
{
    for (size_t index = 0; index < kFrameCount; ++index) {
        lv_image_dsc_t& image = s_descriptors[index];
        image.header.cf = LV_COLOR_FORMAT_RGB565;
        image.header.w = kSize;
        image.header.h = kSize;
        image.header.stride = kSize * sizeof(uint16_t);
        image.data_size = kFrameBytes;
        image.data = kFrameData[index].data();
    }
}

const lv_image_dsc_t* frame(size_t index)
{
    return &s_descriptors[index];
}

}  // namespace refresh_icons
