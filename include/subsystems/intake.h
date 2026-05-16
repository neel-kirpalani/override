#pragma once
#include "main.h"
#include "globals.h"

// Low-level: control motors individually
void intake_main_set_voltage(int mV);
void intake_roller_set_voltage(int mV);

// High-level combined helpers
void intake_set_voltage(int main_mV, int roller_mV);
void intake_stop();

void intake_forward_full();
void intake_reverse_full();
void intake_forward_half();
void intake_reverse_half();
