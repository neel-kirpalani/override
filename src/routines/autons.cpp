#include "main.h"
#include "globals.h"
#include "subsystems/drivetrain.h"
#include "subsystems/intake.h"
#include "routines/autons.h"

// ===== ANGULAR PID TUNE FUNCTION =====
void angular_PID_tune() {
    chassis.setPose(0, 0, 0);
    pros::delay(500);
    chassis.turnToHeading(90, 4000);
    pros::delay(500);
    chassis.turnToHeading(180, 4000);
    pros::delay(500);
    chassis.turnToHeading(0, 4000);
}

// ===== LATERAL PID TUNE FUNCTION =====
void lateral_PID_tune() {
    chassis.setPose(0, 0, 0);
    pros::delay(500);
    chassis.moveToPoint(0, 24, 4000, {.maxSpeed=100});
    
    pros::delay(200);
    chassis.turnToHeading(270, 4000, {.maxSpeed=60});
    pros::delay(500);
    //chassis.setPose(0, 24, 90);
    pros::delay(200);
    chassis.moveToPoint(-24, 24, 4000, {.maxSpeed=60});

}

void auton1(){

}

void auton2(){

}

void auton3(){

}