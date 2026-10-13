#pragma once
#include "main.h"
#include "globals.h"

// Set up the screen and start the refresh task. Call once from initialize().
void screen_init();

// Refresh loop. Call this repeatedly from a background task.
void screen_task();

// Start the driver-period clock. Call once at the top of opcontrol().
void screen_match_start();

