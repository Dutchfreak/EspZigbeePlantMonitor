#pragma once

#include <cstddef>

#include "lvgl.h"

namespace refresh_icons {

constexpr size_t kFrameCount = 4;
constexpr size_t kSize = 16;

void initialize();
const lv_image_dsc_t* frame(size_t index);

}  // namespace refresh_icons
