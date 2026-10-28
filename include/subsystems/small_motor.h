#pragma once
#include "main.h"
#include "globals.h"

// Blocking or asynchronous position helpers for the two 5.5W motors in
// small_motor.
//
// Units are MOTOR-SHAFT ROTATIONS, i.e. full turns of each motor's own output
// shaft as reported by its encoder. They are NOT rotations of the mechanism
// after any external gear ratio. The two ports are -4 and 7, so PROS already
// handles their opposite signs; verify on the robot that positive rotations
// move both sides of the mechanism the same intended way.
//
// All moves use PROS's built-in move_absolute position controller. One
// persistent worker task, started once by small_motor_init(), owns all motor
// commands, stops and monitored move state. It does not run a new PID and does
// not issue repeated target commands. Blocking and async calls share the same
// monitored move state.
//
// Publication/deletion safety: callers never take a mutex. A move is published
// into one of two fixed-size seqlock request buffers, and the published buffer
// is only switched after every field is complete. Every request and ack payload
// word is a lock-free atomic; doubles are bit-copied with memcpy into two
// 32-bit words and reconstructed only after a validated version snapshot. A
// caller deleted mid-write therefore leaves the previous valid request intact
// and the partial buffer is never executed. Waiting for startup, completion and
// cancellation is lock-free. Deleting the current competition task while it is
// submitting, waiting or stopping therefore cannot strand the worker or a later
// small_motor_stop(). Move IDs and cancellation epochs keep stale requests,
// acknowledgements and deadlines from acting on a replacement move. The
// mailbox has one active publisher (the competition task); the worker is the
// only reader, so do not publish helpers concurrently from multiple tasks.
//
// async = false (default): the call validates, publishes its request, and waits
// for its own move. It returns true only if BOTH motors individually reached
// and settled at their own targets. It returns false on timeout, disable,
// invalid input, a motor/read/command error, a newer move replacing it, or
// small_motor_stop(). Existing three-argument calls therefore keep blocking
// behavior.
//
// async = true: the call validates, publishes its request, and waits only for
// the worker to accept BOTH commands. true means the move was accepted/started,
// NOT that it completed. Startup acceptance is published only after both
// commands and a final cancellation/disable/deadline recheck; if startup became
// invalid, both motors are stopped and the call returns false. The worker keeps
// monitoring timeout and settling in the background. A later failure stops both
// small motors and does not abort autonomous. A direct call does not need to
// check the return value.
//
// A newer valid move safely replaces the active one. The replaced move's
// blocking caller returns false, and old worker iterations/deadlines neither
// stop nor report success for the new move.
//
// The timeout budget starts before the first motor command and is monitored in
// the background with rollover-safe elapsed time. Disable and the deadline are
// checked before each command and before success. Motor two is never started
// after the deadline has expired.
//
// On timeout, disable, or a runtime/read/command error, the worker attempts to
// stop BOTH motors with small_motor.brake() (using the existing brake mode) and
// cancels that move's ownership. On success, monitoring ends but the built-in
// position target is left active as before.
//
// Cancellation and raw overrides: call small_motor_stop() to synchronously
// cancel monitoring and stop both motors. Do this before overriding an active
// helper with direct small_motor.move_voltage()/brake() calls. This code does
// not infer raw overrides from telemetry; without the explicit stop, background
// monitoring can later brake the motors. main.cpp calls small_motor_stop() on
// entry to disabled(), autonomous(), and opcontrol().
//
// max_rpm is a positive speed limit in RPM (not a -127..127 voltage/power
// value). For relative moves, the signed rotations argument sets the direction.
// For absolute moves, the direction is the sign of (target position - current
// position), not merely the sign of the target position. The valid range is
// 1..200 RPM because globals.cpp declares small_motor with a green (18:1,
// 200 RPM) cartridge. If that declaration is ever changed, update the limit in
// the .cpp.
//
// Each motor is checked against its own target. The helpers do not average the
// two motors and do not mechanically synchronize them beyond applying the same
// requested motor-shaft delta to each relative move, or the same absolute
// motor-shaft target.
//
// Encoder units: the helpers read the units actually configured on each motor.
// Degrees and rotations are supported. COUNTS is rejected with a message
// because the installed PROS 4.2.1 headers do not define a counts-per-motor-
// shaft-rotation value for these 5.5W motors; guessing one would make the
// public units wrong. Set the motors to degrees or rotations, or add a
// measured counts/rev constant after confirming it on hardware.
//
// Absolute reference: small_motor_move_to() uses the encoder zero in effect
// when it is called. There is no automatic homing and no guarantee that zero
// is a physical home after power-on. initialize() does not zero small_motor.
// To establish a reference deliberately, put the mechanism at a known safe
// position by hand and call, once:
//
//     small_motor.tare_position_all();
//
// That zeros BOTH motors at their current positions for the rest of the
// program run. Do not add that call to initialize() or an auton unless that is
// the intended behavior. A power cycle loses the reference.
//
// Conflict: opcontrol() commands small_motor.move_voltage(...) on every loop
// iteration, sending either 12000 or 0 (0 when L2 is released), every 25 ms.
// That loop therefore overrides these helpers whenever they are called from
// driver control, even with L2 released. Use them in autonomous (or stop that
// command first); this file does not change the driver loop.
//
// Tuning: SMALL_MOTOR_POSITION_TOLERANCE_ROTATIONS and
// SMALL_MOTOR_SETTLE_MS in the .cpp are starting values only. Tune them on the
// real mechanism. Too tight a tolerance can time out because of backlash,
// compliance, or encoder resolution; too loose hides a motor that is not
// actually reaching its target.

// Start the one persistent worker. main.cpp's initialize() calls this; it must
// run before helpers are used. Repeated calls are harmless.
void small_motor_init();

// Synchronously publish a cancellation and wait for the worker to acknowledge
// that BOTH motors have actually been stopped. The acknowledgement is sent only
// after the worker's brake call, so no stale worker write can override driver
// control afterward. If this competition task is deleted while waiting, the
// worker still completes the stop; no mutex is left held.
void small_motor_stop();

// Move each motor by the signed number of motor-shaft rotations, relative to
// that motor's own encoder position at the moment of the call. Both targets are
// captured and tracked separately. A valid zero rotations value is a
// non-commanding no-op that returns true and does not replace an active move.
bool small_motor_move_rotations(double rotations, int max_rpm, int timeout_ms,
                                bool async = false);

// Move each motor to an absolute motor-shaft rotation position, relative to the
// established software zero (see the reference note above).
bool small_motor_move_to(double position_rotations, int max_rpm, int timeout_ms,
                         bool async = false);

// Usage:
//
//     // Async: returns after the commands are accepted; autonomous continues.
//     small_motor_move_rotations(0.25, 50, 2000, true);
//     // Continue other auton commands immediately.
//     pros::delay(150); // Optional timing, not proof of arrival.
//
//     // Fourth argument true selects async. Omit it or pass false to wait.
//     if (!small_motor_move_rotations(2.0, 50, 3000)) {
//         // false: timed out, cancelled, replaced, invalid, or hardware error
//     }
//
//     // Negative relative values reverse direction.
//     if (!small_motor_move_rotations(-1.5, 50, 3000, false)) {
//         // false: timed out, cancelled, replaced, invalid, or hardware error
//     }
//
//     // Absolute moves need a deliberately established zero.
//     if (!small_motor_move_to(5.0, 50, 4000)) {
//         // false: timed out, cancelled, replaced, invalid, or hardware error
//     }
//
// 0.25 is only an example, not a tested safe distance.
