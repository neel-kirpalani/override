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

// 5.5W motors, spun together on L2
extern pros::MotorGroup small_motor;

// Optical sensors
extern pros::Optical colorSensor;
extern pros::Optical sortSensor;

// Pneumatics
extern pros::adi::DigitalOut pneA;
extern pros::adi::DigitalOut pneB;
extern pros::adi::DigitalOut pneD;

// IMU
extern pros::Imu imu;

// Distance Sensors
extern pros::Distance distFront;
extern pros::Distance distLeft;
extern pros::Distance distRight;
