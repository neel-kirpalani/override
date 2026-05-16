#pragma once

#include "api.h"
#include "lemlib/api.hpp"

// Controller
extern pros::Controller controller;

// Drivebase motor groups
extern pros::MotorGroup left_motor_group;
extern pros::MotorGroup right_motor_group;

// Intake motor groups
extern pros::MotorGroup intake_motor;
extern pros::MotorGroup intake_motor2;

// Optical sensors
extern pros::Optical colorSensor;
extern pros::Optical sortSensor;

// Pneumatics
extern pros::adi::DigitalOut pneA;
extern pros::adi::DigitalOut pneB;
extern pros::adi::DigitalOut pneC;

// IMU
extern pros::Imu imu;

// Distance Sensors
extern pros::Distance distFront;
extern pros::Distance distLeft;
extern pros::Distance distRight;
