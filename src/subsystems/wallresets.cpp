#include "wallresets.h"
#include "globals.h"
#include "drivetrain.h"  

// Wall reset constants (inches, path.jerry.io GPS = 72)
const float FIELD_HALF = 72.0;  // half field size (X or Y wall)
const float FRONT_SENSOR_OFFSET = 6.0;  // robot center → front sensor
const float LEFT_SENSOR_OFFSET  = 6.0;  // robot center → left sensor  
const float RIGHT_SENSOR_OFFSET = 6.0;  // robot center → right sensor

// YOUR WALL MAPPING (per description):
// LEFT_WALL  = Y=+72    → distFront  
// RIGHT_WALL = Y=-72    → distFront
// BACK_WALL  = X=-72    → distLeft  
// FRONT_WALL = X=+72    → distRight

void resetToLeftWall() {  // Y=+72 (your left wall, front sensor)
    double d_mm = distFront.get();
    if (d_mm < 20 || d_mm > 2000) return;

    double d_in = d_mm / 25.4;
    lemlib::Pose pose = chassis.getPose();

    double robotY = FIELD_HALF - d_in - FRONT_SENSOR_OFFSET;
    chassis.setPose(pose.x, robotY, pose.theta);
}

void resetToRightWall() {  // Y=-72 (your right wall, front sensor)  
    double d_mm = distFront.get();
    if (d_mm < 20 || d_mm > 2000) return;

    double d_in = d_mm / 25.4;
    lemlib::Pose pose = chassis.getPose();

    double robotY = -FIELD_HALF + d_in + FRONT_SENSOR_OFFSET;
    chassis.setPose(pose.x, robotY, pose.theta);
}

void resetToBackWall() {   // X=-72 (your back wall, left sensor)
    double d_mm = distLeft.get();
    if (d_mm < 20 || d_mm > 2000) return;

    double d_in = d_mm / 25.4;
    lemlib::Pose pose = chassis.getPose();

    double robotX = -FIELD_HALF + d_in + LEFT_SENSOR_OFFSET;
    chassis.setPose(robotX, pose.y, pose.theta);
}

void resetToFrontWall() {  // X=+72 (your front wall, right sensor)
    double d_mm = distRight.get();
    if (d_mm < 20 || d_mm > 2000) return;

    double d_in = d_mm / 25.4;
    lemlib::Pose pose = chassis.getPose();

    double robotX = FIELD_HALF - d_in - RIGHT_SENSOR_OFFSET;
    chassis.setPose(robotX, pose.y, pose.theta);
}
