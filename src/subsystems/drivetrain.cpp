#include "main.h"
#include "lemlib/api.hpp"
#include "globals.h"
#include "drivetrain.h"

// Drivetrain config
lemlib::Drivetrain drivetrain(
    &left_motor_group,               // left motor group
    &right_motor_group,              // right motor group
    11.396,                          // track width
    lemlib::Omniwheel::NEW_325,      // wheel type
    450,                             // drivetrain rpm
    2                                // horizontal drift
);

// Odometry sensors
lemlib::OdomSensors odomSensors(
    nullptr,  // vertical tracking wheel 1
    nullptr,  // vertical tracking wheel 2
    nullptr,  // horizontal tracking wheel 1
    nullptr,  // horizontal tracking wheel 2
    &imu      // inertial sensor
);

// Lateral PID controller
lemlib::ControllerSettings lateral_controller(
    10,   // kP
    0,   // kI
    20,   // kD
    0,   // anti windup
    0,   // small error range (in)
    0,   // small error timeout (ms)
    0,   // large error range (in)
    0,   // large error timeout (ms)
    0    // max acceleration
);

// Angular PID controller
lemlib::ControllerSettings angular_controller(
    2.5,   // kP
    0,   // kI
    18,  // kD
    0,   // anti windup
    0,   // small error range (deg)
    0,   // small error timeout (ms)
    0,   // large error range (deg)
    0,   // large error timeout (ms)
    40    // max acceleration
);

// Global chassis object
lemlib::Chassis chassis(
    drivetrain,
    lateral_controller,
    angular_controller,
    odomSensors
);
