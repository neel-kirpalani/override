#include "main.h"
#include "globals.h"
#include "subsystems/cascade.h"

#include <cstdio>

// How often the lift control loop runs. 10 ms = 100 Hz, which is the rate the
// PD gains were tuned against.
static const int CONTROL_PERIOD_MS = 10;

// ===== Speeds =====
const int CASCADE_VOLTAGE = 12000;  // manual R1/R2 speed (full power)

// ===== Return to bottom =====
const int HOME_VOLTAGE = -6000;     // slow unwind when returning to the bottom
const int HOME_TIMEOUT_MS = 4000;   // stop anyway after this long

// ===== Go to position (autonomous) =====
// Used by cascade_go_to(). Kept slower than manual so it does not overshoot.
const int GOTO_VOLTAGE = 10000;     // max output while driving to a target
const double GOTO_TOL = 20;         // "close enough" in encoder degrees
const int GOTO_TIMEOUT_MS = 4000;   // give up after this long

// ===== State =====
// bottom_pos is captured at program start. That is our zero reference.
double bottom_pos = 0;

// Encoder value of each cascade motor at startup. Raw positions cannot be
// averaged directly when one port is reversed (a "-15" reports negative counts
// as the lift rises), because equal and opposite readings cancel to zero.
// Normalising every motor against its own baseline first makes the signs agree.
static double motor_base[8] = {0};
static int    motor_base_n  = 0;

// Return-to-bottom state
bool homing = false;
// True once a return-to-bottom has completed at least once. Motion is still
// accepted without it, but go-to positions are meaningless until the zero is
// established, so the UI surfaces it.
bool has_homed = false;
uint32_t home_start = 0;
double home_target = 0;

// Go-to-position state
bool going = false;
bool move_timed_out = false;   // set when a move gave up rather than arriving
double goto_target = 0;
uint32_t goto_start = 0;

void cascade_init() {
    cascade_motor.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

    // Whatever position the cascade is in at program start is our bottom.
    // Every recorded position is relative to this, so always start with the
    // cascade at the bottom or the numbers will be shifted.
    // Capture each motor's starting encoder value BEFORE anything moves.
    // Everything below is measured relative to these.
    {
        std::vector<double> b = cascade_motor.get_position_all();
        motor_base_n = 0;
        for (size_t i = 0; i < b.size() && motor_base_n < 8; i++) {
            motor_base[motor_base_n++] = b[i];
        }
    }

    bottom_pos = cascade_position();

    // Boot diagnostic: print each cascade motor's encoder at startup. If a
    // port is wrong, unplugged, or the encoder is dead, it shows here before
    // anything moves. Open the PROS terminal to see it.
    std::vector<double> all = cascade_motor.get_position_all();
    printf("[cascade] %d motor(s) at boot:", (int)all.size());
    for (size_t i = 0; i < all.size(); i++) {
        printf("  %.0f", all[i]);
    }
    printf("   (averaged %.0f)\n", bottom_pos);

    // Start the lift control loop on its own task. From here on the cascade
    // keeps moving in the background, so autonomous can drive while it raises
    // instead of waiting for it.
    pros::Task cascadeTask([]() {
        cascade_task();
    }, TASK_PRIORITY_DEFAULT, 4096, "cascade");
}

// Current cascade position, in encoder degrees.
//
// This is the value to read off the brain and pass to cascade_go_to().
//
// AVERAGES every motor in the group rather than reading motor 0 only: if one
// motor is unplugged, mis-wired, or its encoder has failed, a single-motor
// read can sit at zero (or return an error) while the cascade is actually
// moving. Averaging the healthy motors degrades gracefully instead.
// One motor's movement since startup, in encoder degrees, with the port's
// direction normalised so a reversed port ("-15") counts the same way as a
// forward one.
//
// The sign is taken from each motor's own baseline rather than from the
// movement, so the result keeps its direction: it rises when the cascade
// rises and falls when the cascade lowers. Careful: taking the magnitude of
// the movement instead would make the value only ever climb, which breaks
// both the readout and cascade_go_to()'s idea of "above" and "below".
static bool motor_delta(const std::vector<double>& all, size_t i, double* out) {
    if (i >= all.size()) return false;
    double v = all[i];

    // Reject unusable readings. PROS returns PROS_ERR_F (a large negative) on
    // error, and a bad read can also surface as NaN or infinity.
    if (!(v == v)) return false;      // NaN
    if (v <= -1.0e9) return false;    // PROS_ERR_F
    if (v > 1.0e9) return false;      // +inf / nonsense

    double base = (i < (size_t)motor_base_n) ? motor_base[i] : 0.0;

    // A negative baseline means this port reports inverted, so flip it.
    double sign = (base < 0.0) ? -1.0 : 1.0;

    *out = sign * (v - base);
    return true;
}

double cascade_position() {
    std::vector<double> all = cascade_motor.get_position_all();

    double sum = 0.0;
    int n = 0;
    for (size_t i = 0; i < all.size(); i++) {
        double d;
        if (!motor_delta(all, i, &d)) continue;
        sum += d;
        n++;
    }
    if (n == 0) return 0.0;
    return sum / (double)n;
}

// Per-motor encoder values, for diagnosing a single bad motor on the brain.
// Returns the number of motors written.
// How many motors the group actually claims. If this is lower than expected,
// a port in globals.cpp is not being recognised.
int cascade_motor_count() {
    return (int)cascade_motor.get_position_all().size();
}

// The raw position of motor 0 only, with no averaging. If the averaged value
// is 0 while this is not, the averaging is the problem; if both are 0, the
// encoder itself is reporting nothing.
double cascade_position_raw() {
    return cascade_motor.get_position();
}

// The baseline captured at startup, per motor, for display.
int cascade_baselines(double* out, int max) {
    int n = 0;
    for (int i = 0; i < motor_base_n && n < max; i++, n++) out[n] = motor_base[i];
    return n;
}

int cascade_motor_positions(double* out, int max) {
    std::vector<double> all = cascade_motor.get_position_all();
    int n = 0;
    for (size_t i = 0; i < all.size() && n < max; i++) {
        double d;
        if (!motor_delta(all, i, &d)) continue;
        out[n++] = d;      // normalised, so a reversed port reads the same way
    }
    return n;
}

void cascade_up() {
    cascade_motor.move_voltage(CASCADE_VOLTAGE);
}

void cascade_down() {
    cascade_motor.move_voltage(-CASCADE_VOLTAGE);
}

void cascade_stop() {
    cascade_motor.move_voltage(0);
}

// Start returning to the bottom from wherever we are
void cascade_home() {
    homing = true;
    going = false;                  // a go-to move is cancelled
    home_start = pros::millis();
    home_target = bottom_pos;
    cascade_motor.move_voltage(HOME_VOLTAGE);
}

bool cascade_homing() {
    return homing;
}

// True once the lift has homed at least once since the program started.
bool cascade_has_homed() {
    return has_homed;
}

// True while a go-to move is in progress.
bool cascade_moving() {
    return going;
}

// Drive to a recorded motor position. Used in autonomous, for example:
//     cascade_go_to(1450);
//     cascade_wait();
void cascade_go_to(double position) {
    going = true;
    move_timed_out = false;
    homing = false;                 // a return-to-bottom is cancelled
    goto_target = position;
    goto_start = pros::millis();
}

// True once the cascade has reached its target (or given up)
bool cascade_at_target() {
    return !going;
}

// Block until the cascade reaches its target. Returns false if it timed out.
// Drives the move itself, so autonomous only needs:
//     cascade_go_to(1450);
//     cascade_wait();
bool cascade_wait() {
    // The control loop is driven by cascade_task(), so this only waits.
    // Returns false if the move timed out rather than arriving.
    while (going) {
        pros::delay(10);
    }
    return !move_timed_out;
}

// Non-blocking: true while a go-to move is still running.
bool cascade_busy() {
    return going;
}

// The lift control loop, run on its own task so the cascade keeps moving
// while the rest of the robot does something else - driving, turning, or
// running an intake. Nothing has to call cascade_update() from the main flow.
void cascade_task() {
    while (true) {
        cascade_update();
        pros::delay(CONTROL_PERIOD_MS);
    }
}

// Call every loop. Runs whichever move is active.
void cascade_update() {
    uint32_t now = pros::millis();
    double pos = cascade_position();

    // ---- Return to bottom ----
    if (homing) {
        bool arrived = pos <= home_target;
        bool timed_out = (now - home_start) >= HOME_TIMEOUT_MS;

        if (arrived || timed_out) {
            cascade_stop();
            homing = false;
            has_homed = true;
        }
        return;
    }

    // ---- Go to a recorded position ----
    if (going) {
        double error = goto_target - pos;

        // Close enough: stop and let brake hold
        if (fabs(error) <= GOTO_TOL) {
            cascade_stop();
            going = false;
            return;
        }

        bool timed_out = (now - goto_start) >= GOTO_TIMEOUT_MS;
        if (timed_out) {
            cascade_stop();
            going = false;
            move_timed_out = true;
            return;
        }

        // Slow down as we get close so we do not sail past the target
        int voltage = GOTO_VOLTAGE;
        if (fabs(error) < GOTO_TOL * 5) {
            voltage = GOTO_VOLTAGE / 2;
        }

        if (error > 0) {
            cascade_motor.move_voltage(voltage);    // target is above us
        } else {
            cascade_motor.move_voltage(-voltage);   // target is below us
        }
    }
}
