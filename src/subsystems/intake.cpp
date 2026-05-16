#include "main.h"
#include "globals.h"
#include "subsystems/intake.h"

// Individual control
void intake_main_set_voltage(int mV) {
    intake_motor.move_voltage(mV);      // main intake group
}

void intake_roller_set_voltage(int mV) {
    intake_motor2.move_voltage(mV);     // roller / second intake
}

// Combined helper
void intake_set_voltage(int main_mV, int roller_mV) {
    intake_main_set_voltage(main_mV);
    intake_roller_set_voltage(roller_mV);
}

void intake_stop() {
    intake_set_voltage(0, 0);
}

void intake_forward_full() {
    intake_set_voltage(12000, 12000);
}

void intake_reverse_full() {
    intake_set_voltage(-12000, -12000);
}

void intake_forward_half() {
    intake_set_voltage(6000, 6000);
}

void intake_reverse_half() {
    intake_set_voltage(-6000, -6000);
}
