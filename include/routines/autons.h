#pragma once
#include "main.h"
#include "globals.h"
#include "subsystems/drivetrain.h"
#include "subsystems/intake.h"

// Tuning routines
void angular_PID_tune();
void lateral_PID_tune();

// Match autons
void auton1();
void auton2();
void auton3();
