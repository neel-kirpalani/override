#pragma once
#include "liblvgl/lvgl.h"

// The Override field, rendered from the standardised orthographic view at
// field-rendering.jerryio.com (CC BY 4.0) and embedded as an LVGL image.
//
// The AUTON page draws this as its background and overlays the live robot
// marker on top, so the field geometry is exact rather than approximated.
//
// Defined in src/subsystems/field_img.cpp (auto-generated from the field render).
extern const lv_image_dsc_t field_img;
