#pragma once
#include "main.h"
#include "globals.h"
#include "subsystems/drivetrain.h"

// Tuning routines
void angular_PID_tune();
void lateral_PID_tune();

// Match autons
void three_pin();
void two_wall_pins();
void secondary_one_pin();
void primary_one_pin();
