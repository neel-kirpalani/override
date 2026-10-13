#include "main.h"
#include "globals.h"
#include "subsystems/drivetrain.h"
#include "subsystems/intake.h"
#include "routines/skills.h"
#include "subsystems/cascade.h"

void skills_auton() {
    chassis.setPose(-63.65, 0, 90);
    chassis.moveToPoint(-59, 0, 900);
    pros::delay(10);
    chassis.moveToPoint(-63.65, 0, 900);
    pros::delay(20);
    chassis.moveToPoint(-59, 0, 900);
    pros::delay(10);
    chassis.moveToPoint(-63.65, 0, 900);
    pros::delay(20);
    chassis.moveToPoint(-47.041, 0, 1000);
    pros::delay(100);
    chassis.turnToHeading(180, 900, {}, true);
    pros::delay(50);
    cascade_go_to(50);
    chassis.waitUntilDone();
    chassis.moveToPoint(-47.041, 0, 1000, {.maxSpeed=50});
    pros::delay(100);
    cascade_go_to(0);
    pros::delay(100);
    pneA.set_value(true);
    pros::delay(100);
    chassis.moveToPoint(-47.171, -5.564, 1000);
    pros::delay(20);
    chassis.turnToHeading(128, 900);
    pros::delay(10);
    chassis.moveToPoint(-23.469, -23.98, 1000, {.maxSpeed=50});
    pros::delay(10);
    pneA.set_value(false);
    pros::delay(40);
    chassis.turnToHeading(270, 900, {}, true);
    pros::delay(15);
    cascade_go_to(150);
    chassis.waitUntilDone();
    chassis.moveToPoint(-39, -23.98, 900, {.maxSpeed=60});
    pros::delay(50);
    cascade_go_to(0);
    pros::delay(200);
    pneA.set_value(true);
    pros::delay(100);
    chassis.moveToPoint(-23.469, -23.98, 1000, {.forwards=false});
    pros::delay(20);
    chassis.turnToHeading(225, 900);
    pros::delay(10);
    chassis.moveToPoint(-42.429, -42.94, 1000, {.maxSpeed=60});
    pros::delay(100);
    pneA.set_value(false);
    pros::delay(100);
    chassis.swingToHeading(0, DriveSide::RIGHT, 1000, {}, true);
    pros::delay(20);
    cascade_go_to(350);
    chassis.waitUntilDone();
    chassis.moveToPoint(-47.154, -30.507, 1000, {.maxSpeed=60});
    pros::delay(100);
    cascade_go_to(0);
    pros::delay(200);
    pneA.set_value(true);
    pros::delay(200);
    chassis.moveToPoint(-47.154, -59.117, 1200, {.forwards=false});
    pros::delay(50);
    chassis.turnToHeading(270, 900);
    pros::delay(100);
    chassis.moveToPoint(-61.671, -59.117, 1000, {.maxSpeed=40});
    pros::delay(400);
    pneA.set_value(false);
    pros::delay(500);
    chassis.moveToPoint(-47.154, -59.117, 1200, {.forwards=false});
    pros::delay(200);
    chassis.turnToHeading(0, 900, {}, true);
    pros::delay(15);
    cascade_go_to(430);
    chassis.waitUntilDone();
    pros::delay(100);
    chassis.moveToPoint(-47.154, -30.507, 1200, {.maxSpeed=50});
    pros::delay(100);
    cascade_go_to(0);
    pros::delay(500);
    pneA.set_value(true);
    









}
