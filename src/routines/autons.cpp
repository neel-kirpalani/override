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
    /*
    pros::delay(500);
    chassis.turnToHeading(180, 4000);
    pros::delay(500);
    chassis.turnToHeading(0, 4000);
    */
}

// ===== LATERAL PID TUNE FUNCTION =====
void lateral_PID_tune() {
    
    chassis.setPose(0, 0, 0);
    pros::delay(500);
    chassis.moveToPoint(0, 48, 10000);
   
    /*
    pros::delay(500);
    chassis.turnToHeading(180, 1000);
    pros::delay(500);
    chassis.moveToPoint(0,0,3000);
    */

    

    /*
    pros::delay(200);
    chassis.turnToHeading(270, 4000, {.maxSpeed=60});
    pros::delay(500);
    //chassis.setPose(0, 24, 90);
    pros::delay(200);
    chassis.moveToPoint(-24, 24, 4000, {.maxSpeed=60});
    */
}

void left4_1(){
    chassis.setPose(-9.167,-60.208,132);
    chassis.moveToPoint(-17.728, -52.478, 3000, {.forwards=false,.maxSpeed=50});
    pros::delay(1000);
    chassis.moveToPoint(-14.305, -55.404, 2000, {.forwards=true, .maxSpeed=50});
    pros::delay(500);
    chassis.turnToHeading(270, 1000);
    pros::delay(500);
    chassis.moveToPoint(-4.248, -55.404, 2000, {.maxSpeed=50});
    pros::delay(500);
    chassis.turnToHeading(180,1000);
    pros::delay(500);
    chassis.moveToPoint(-4.248, -64.92, 4000, {.maxSpeed=30});
    chassis.waitUntilDone();
    pros::delay(100);
    chassis.moveToPoint(-4.248, -60.208, 2000, {.forwards=false, .maxSpeed=50});
    pros::delay(200);
    chassis.moveToPoint(-4.248, -64.92, 4000, {.maxSpeed=30});
    chassis.waitUntilDone();
    pros::delay(200);
    chassis.moveToPoint(-4.248, -55.404, 2000, {.maxSpeed=50});
    pros::delay(500);
    chassis.turnToHeading(270,1000);
    pros::delay(500);
    chassis.moveToPoint(-23.5, -55.404, 2000, {.maxSpeed=50});
    pros::delay(500);
    chassis.turnToHeading(180,1000);
    pros::delay(500);
    chassis.moveToPoint(-23.5, -62.278, 4000, {.maxSpeed=30});
    pros::delay(1000);
    chassis.moveToPoint(-23.5, -54.595, 3000, {.maxSpeed=30});
    pros::delay(500);
    chassis.moveToPoint(-23.5, -55.404, 2000, {.maxSpeed=30});
    pros::delay(500);
    chassis.turnToHeading(270, 1000);
    pros::delay(500);
    chassis.moveToPoint(-35.213, -55.404, 2000);
    pros::delay(500);
    chassis.turnToHeading(305, 1000);
    pros::delay(500);
    chassis.moveToPoint(-41.273, -51.076, 2000);
    pros::delay(500);




}

void auton2(){

}

void auton3(){

}