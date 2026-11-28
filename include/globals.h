#pragma once

#include "api.h"
#include "lemlib/api.hpp"

// Controller
extern pros::Controller controller;

// Drivebase motor groups
extern pros::MotorGroup left_motor_group;
extern pros::MotorGroup right_motor_group;

// Cascade motor group
extern pros::MotorGroup cascade_motor;


// Optical sensors
extern pros::Optical colorSensor;
extern pros::Optical sortSensor;

// Pneumatics
extern pros::adi::DigitalOut pneA;

// IMU
extern pros::Imu imu;

// Distance Sensors
extern pros::Distance distFront;
extern pros::Distance distLeft;
extern pros::Distance distRight;
