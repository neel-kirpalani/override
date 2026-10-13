#pragma once
#include "main.h"
#include "globals.h"

void cascade_init();

// Read the cascade position (encoder degrees, 0 at program start). This is
// the value to pass to cascade_go_to().
double cascade_position();

// Per-motor encoder values, for spotting one bad motor. Returns how many were
// written. Pass an array of at least 3.
int cascade_motor_positions(double* out, int max);

// Diagnostics: how many motors the group claims, and the raw motor-0 reading
// with no averaging. Used by the STATUS page to tell an encoder problem apart
// from a code problem.
int cascade_motor_count();
double cascade_position_raw();
int cascade_baselines(double* out, int max);

// Hold R1 to raise, hold R2 to lower
void cascade_up();
void cascade_down();
void cascade_stop();

// Press X to return the cascade to the bottom from any height
void cascade_home();
void cascade_update();
bool cascade_homing();

// True once the lift has homed at least once this program run. Until then,
// recorded go-to positions are relative to an unset zero.
bool cascade_has_homed();

// True while a go-to move is in progress.
bool cascade_moving();

// Drive to a recorded motor position (used by autonomous routines).
// This only sets a target - cascade_task() does the driving.
void cascade_go_to(double position);

// True once the move has finished (arrived or given up).
bool cascade_at_target();

// True while a move is still running. Use this to drive and raise at the same
// time instead of waiting.
bool cascade_busy();

// Block until the move finishes. Returns false if it timed out.
bool cascade_wait();

// The lift control loop. Runs on its own task, started by cascade_init(),
// so the cascade keeps moving while the robot does other things.
void cascade_task();
