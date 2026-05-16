#include "subsystems/macros.h"

static bool intake_macro_active = false;

constexpr int INTAKE_HALF_MAIN  = 6000;   // custom speed for main
constexpr int INTAKE_HALF_ROLL  = 6000;   // custom speed for roller

void macro_intake_half(bool enabled) {
    if (enabled) {
        intake_set_voltage(INTAKE_HALF_MAIN, INTAKE_HALF_ROLL);
    }
    // Non-destructive when disabled: do nothing.
}
