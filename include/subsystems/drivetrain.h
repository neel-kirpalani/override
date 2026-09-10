#pragma once
#include "main.h"
#include "lemlib/api.hpp"
#include "globals.h"

// Expose the chassis so other files can use it
extern lemlib::Chassis chassis;

// Optionally expose these if you ever tune them from other files:
extern lemlib::Drivetrain drivetrain;
extern lemlib::OdomSensors odomSensors;
extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;

