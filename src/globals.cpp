#include "main.h"
#include "pros/motors.h"
#include "pros/optical.h"
#include "pros/adi.h"
#include "pros/imu.h"
#include "globals.h"

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Drivebase 
pros::MotorGroup left_motor_group({14,-13}, pros::MotorGearset::blue);
pros::MotorGroup right_motor_group({19, -20}, pros::MotorGearset::blue);

// Cascade
pros::MotorGroup cascade_motor({14, -15}, pros::MotorGearset::green, pros::MotorUnits::degrees);

// Pneumatics 
pros::adi::DigitalOut claw_angler('A');
pros::adi::DigitalOut toggle_holder('B');

// IMU 
pros::Imu imu(17);

// Distance Sensors



