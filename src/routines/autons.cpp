#include "main.h"
#include "globals.h"
#include "subsystems/drivetrain.h"
#include "subsystems/intake.h"
#include "routines/autons.h"
#include "subsystems/cascade.h"

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
    chassis.moveToPoint(0, 24, 10000);
   
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

void left4_1(){ // not tested
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

void auton2(){ // unfinished
    chassis.setPose(-63.65, 0, 90);
    chassis.moveToPoint(-57.015, 0, 1200, {.maxSpeed=30});
    pros::delay(100);
    chassis.turnToHeading(21, 900);
    pros::delay(100);
    chassis.moveToPoint(-50.734, 15.732, 1200, {.maxSpeed=50});
    pros::delay(100);
    chassis.moveToPoint(-53.593, 8.285, 1200, {.forwards=false, .maxSpeed=30});
    pros::delay(100);
    chassis.turnToHeading(330, 900);
    pros::delay(100);
    chassis.moveToPoint(-59.946, 19.288, 1200, {.maxSpeed=60});
    pros::delay(100);
    chassis.swingToHeading(270, DriveSide::LEFT, 1000); 
    pros::delay(100);
    chassis.swingToPoint(-53.593, 8.285, DriveSide::RIGHT, 900, {.forwards=false});
    pros::delay(100);
    chassis.moveToPoint(-53.593, 8.285, 2000, {.forwards=false, .maxSpeed=70});
    pros::delay(1000);
    chassis.turnToHeading(21, 900);
    pros::delay(100);
}

void auton3(){
    chassis.setPose(63.65, 0, 270);
    chassis.moveToPoint(59, 0, 1200, {.maxSpeed=80});
    pros::delay(10);
    chassis.moveToPoint(65, 0, 750, {.forwards=false, .minSpeed=100});
    pros::delay(20);
    chassis.moveToPoint(59, 0, 750, {.maxSpeed=80});
    pros::delay(10);
    chassis.moveToPoint(65, 0, 800, {.forwards=false,.minSpeed=100});
    pros::delay(20);
    chassis.moveToPoint(58, 0, 800, {.minSpeed=80}, true);
    pros::delay(10);
    cascade_go_to(50);
    chassis.waitUntilDone();
    chassis.turnToHeading(339, 800);
    pros::delay(50);
    chassis.moveToPoint(50.734, 15.732, 1200, {.maxSpeed=50});
    pros::delay(10);
    cascade_go_to(0);
    pneA.set_value(true);
    pros::delay(100);
    chassis.moveToPoint(54.256, 6.556, 900, {.forwards=false, .maxSpeed=80});
    pros::delay(30);
    chassis.turnToHeading(298.5, 900);
    pros::delay(20);
    chassis.moveToPoint(41.374, 13.566, 900, {.maxSpeed=100});
    pros::delay(5);
    chassis.moveToPoint(32.358, 18.446, 900, {.maxSpeed=50});
    pros::delay(10);
    pneA.set_value(false);
    pros::delay(50);
    chassis.turnToHeading(75, 900, {}, true);
    cascade_go_to(200);
    chassis.waitUntilDone();
    pros::delay(20);
    chassis.moveToPoint(39.124, 21.265, 1000, {.maxSpeed=60});
    pros::delay(50);
    cascade_go_to(0);
    pros::delay(100);
    pneA.set_value(true);
    pros::delay(100);
    chassis.moveToPoint(23.38, 23.391, 1200, {.forwards=false,.maxSpeed=120});
    pros::delay(10);
    chassis.turnToHeading(45, 900);
    pros::delay(20);
    chassis.moveToPoint(35.586, 35.597, 900, {.maxSpeed=100});
    pros::delay(5);
    chassis.moveToPoint(42.824, 42.835, 1200, {.maxSpeed=50});
    pros::delay(10);
    pneA.set_value(false);
    pros::delay(40);
    chassis.turnToHeading(175, 780, {}, true);
    pros::delay(30);
    cascade_go_to(350);
    chassis.waitUntilDone();
    pros::delay(50);
    chassis.moveToPoint(45.101, 30.429, 1200, {.maxSpeed=50});
    pros::delay(40);
    cascade_go_to(0);
    pros::delay(100);
}
