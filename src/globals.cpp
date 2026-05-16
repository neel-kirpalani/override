#include "main.h"
#include "pros/motors.h"
#include "pros/optical.h"
#include "pros/adi.h"
#include "pros/imu.h"
#include "globals.h"

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Drivebase 
pros::MotorGroup left_motor_group({-1,-2,-3}, pros::MotorGearset::blue);
pros::MotorGroup right_motor_group({11, 12, 13}, pros::MotorGearset::blue);

// Intakes 
pros::MotorGroup intake_motor({9, -8}, pros::MotorGearset::blue);
pros::MotorGroup intake_motor2({-7}, pros::MotorGearset::blue);

// Sensors 


// Pneumatics 
pros::ADIDigitalOut pneA('A');
pros::ADIDigitalOut pneB('B');
pros::ADIDigitalOut pneC('C');

// IMU 
pros::Imu imu(21);

// Distance Sensors



