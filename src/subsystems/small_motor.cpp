#include "main.h"
#include "globals.h"
#include "subsystems/small_motor.h"

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

namespace {

// Position tolerance in MOTOR-SHAFT ROTATIONS. Each motor must be within this
// distance of its own target. TUNE ON THE ROBOT: 0.02 rev is 7.2 degrees of
// motor shaft, which is a reasonable starting point for the green 5.5W motor.
// Backlash or a compliant mechanism may need a looser value; do not tighten it
// until you have confirmed both motors can actually hold the target.
constexpr double SMALL_MOTOR_POSITION_TOLERANCE_ROTATIONS = 0.02;

// Both motors must remain inside tolerance continuously for this long before
// the helper reports success. TUNE ON THE ROBOT; this just rejects a single
// momentary pass through the target while the motors are still settling.
constexpr int SMALL_MOTOR_SETTLE_MS = 100;

// Worker monitor period, and the short yield used by lock-free callers while
// they wait for an acknowledgement.
constexpr int SMALL_MOTOR_POLL_MS = 10;
constexpr int SMALL_MOTOR_WAIT_MS = 1;

// small_motor is declared with pros::MotorGearset::green in globals.cpp, which
// is the fixed 200 RPM gearing of these 5.5W motors. This is the configured
// hardware limit used to validate max_rpm.
constexpr int SMALL_MOTOR_MAX_RPM = 200;

constexpr int SMALL_MOTOR_COUNT = 2;
constexpr int SMALL_MOTOR_REQUEST_BUFFERS = 2;
constexpr int SMALL_MOTOR_ACK_SLOTS = 8;

enum class StartupStatus : std::uint8_t {
	pending = 0,
	accepted = 1,
	rejected = 2,
};

enum class ResultStatus : std::uint8_t {
	none = 0,
	success = 1,
	failure = 2,
	replaced = 3,
	cancelled = 4,
};

// Two request buffers. A caller writes only the buffer that is NOT currently
// published, then publishes it with one atomic store. If the caller is deleted
// mid-write, the old published request remains intact and the partial buffer is
// never executed. Every payload word is a lock-free atomic, so a reader that
// overlaps a writer has no ordinary data race; the seqlock version decides
// whether the snapshot is usable.
struct RequestBuffer {
	std::atomic<std::uint32_t> seq{0};
	std::atomic<std::uint32_t> id{0};
	std::atomic<std::uint32_t> epoch{0};
	std::atomic<std::uint32_t> target_words[SMALL_MOTOR_COUNT * 2];
	std::atomic<std::uint32_t> tolerance_words[SMALL_MOTOR_COUNT * 2];
	std::atomic<std::uint32_t> max_rpm{0};
	std::atomic<std::uint32_t> timeout_ms{0};

	RequestBuffer() {
		for (int i = 0; i < SMALL_MOTOR_COUNT * 2; ++i) {
			target_words[i].store(0, std::memory_order_seq_cst);
			tolerance_words[i].store(0, std::memory_order_seq_cst);
		}
	}
};

RequestBuffer g_request_buffers[SMALL_MOTOR_REQUEST_BUFFERS];
std::atomic<int> g_published_buffer{-1};

// Worker-written acknowledgements. Callers scan by request ID. Every payload
// field is atomic for the same reason as the request payload.
struct AckSlot {
	std::atomic<std::uint32_t> seq{0};
	std::atomic<std::uint32_t> id{0};
	std::atomic<std::uint32_t> startup{0};
	std::atomic<std::uint32_t> result{0};
};

AckSlot g_acks[SMALL_MOTOR_ACK_SLOTS];
std::atomic<std::uint32_t> g_ack_ticket{0};

std::atomic<std::uint32_t> g_next_request_id{0};

// Incremented by small_motor_stop(). The worker acknowledges each epoch only
// after it has actually stopped both motors. Competition tasks never hold a
// mutex here, so their deletion cannot strand the worker or a later stop.
std::atomic<std::uint32_t> g_cancel_epoch{0};
std::atomic<std::uint32_t> g_stop_ack_epoch{0};

std::atomic<bool> g_worker_created{false};
std::atomic<bool> g_worker_ready{false};

// The lock-free handoff relies on these being single-instruction atomics on this
// target. If this assertion ever fails, the design needs revisiting rather than
// silently using a lock that a deleted competition task could hold.
static_assert(std::atomic<std::uint32_t>::is_always_lock_free,
              "small_motor requires lock-free 32-bit atomics");
static_assert(std::atomic<int>::is_always_lock_free,
              "small_motor requires lock-free 32-bit atomics");
static_assert(std::atomic<bool>::is_always_lock_free,
              "small_motor requires lock-free bool atomics");
static_assert(sizeof(double) == 2 * sizeof(std::uint32_t),
              "small_motor stores doubles as two 32-bit words");

// Worker-owned state. Only the persistent worker touches these fields.
bool g_active = false;
std::uint32_t g_active_id = 0;
std::uint32_t g_active_epoch = 0;
std::uint32_t g_active_start_ms = 0;
int g_active_timeout_ms = 0;
double g_active_targets[SMALL_MOTOR_COUNT] = {0.0, 0.0};
double g_active_tolerance[SMALL_MOTOR_COUNT] = {0.0, 0.0};
bool g_active_within = false;
std::uint32_t g_active_within_since = 0;
std::uint32_t g_last_processed_id = 0;
std::uint32_t g_last_stop_ack = 0;

struct RequestData {
	bool valid = false;
	std::uint32_t id = 0;
	std::uint32_t epoch = 0;
	double targets[SMALL_MOTOR_COUNT] = {0.0, 0.0};
	double tolerance[SMALL_MOTOR_COUNT] = {0.0, 0.0};
	int max_rpm = 0;
	int timeout_ms = 0;
};

bool id_newer(std::uint32_t a, std::uint32_t b) {
	return static_cast<std::int32_t>(a - b) > 0;
}

void store_double_words(std::atomic<std::uint32_t>* words, double value) {
	std::uint32_t bits[2];
	std::memcpy(bits, &value, sizeof(bits));
	words[0].store(bits[0], std::memory_order_seq_cst);
	words[1].store(bits[1], std::memory_order_seq_cst);
}

double reconstruct_double(const std::uint32_t* words) {
	double value;
	std::memcpy(&value, words, sizeof(value));
	return value;
}

std::uint32_t begin_write(std::atomic<std::uint32_t>& seq) {
	const std::uint32_t s = seq.load(std::memory_order_seq_cst);
	// If s is odd, a previous writer was deleted mid-write. Start a fresh odd
	// sequence; the partial data is never accepted.
	const std::uint32_t w = (s & 1u) ? (s + 2u) : (s + 1u);
	seq.store(w, std::memory_order_seq_cst);
	return w;
}

void end_write(std::atomic<std::uint32_t>& seq, std::uint32_t w) {
	seq.store(w + 1u, std::memory_order_seq_cst);
}

bool read_published_request(RequestData* out) {
	const int index = g_published_buffer.load(std::memory_order_seq_cst);
	if (index < 0 || index >= SMALL_MOTOR_REQUEST_BUFFERS) {
		return false;
	}

	RequestBuffer& buffer = g_request_buffers[index];
	const std::uint32_t s1 = buffer.seq.load(std::memory_order_seq_cst);
	if (s1 & 1u) {
		return false;
	}

	const std::uint32_t id = buffer.id.load(std::memory_order_seq_cst);
	const std::uint32_t epoch = buffer.epoch.load(std::memory_order_seq_cst);
	const std::uint32_t max_rpm = buffer.max_rpm.load(std::memory_order_seq_cst);
	const std::uint32_t timeout_ms =
	    buffer.timeout_ms.load(std::memory_order_seq_cst);

	std::uint32_t target_bits[SMALL_MOTOR_COUNT * 2];
	std::uint32_t tolerance_bits[SMALL_MOTOR_COUNT * 2];
	for (int i = 0; i < SMALL_MOTOR_COUNT * 2; ++i) {
		target_bits[i] = buffer.target_words[i].load(std::memory_order_seq_cst);
		tolerance_bits[i] =
		    buffer.tolerance_words[i].load(std::memory_order_seq_cst);
	}

	const std::uint32_t s2 = buffer.seq.load(std::memory_order_seq_cst);
	if (s1 != s2 || id == 0) {
		return false;
	}

	// Reconstruct only after the version snapshot proved consistent.
	RequestData data;
	data.id = id;
	data.epoch = epoch;
	data.max_rpm = static_cast<int>(max_rpm);
	data.timeout_ms = static_cast<int>(timeout_ms);
	for (int i = 0; i < SMALL_MOTOR_COUNT; ++i) {
		data.targets[i] = reconstruct_double(&target_bits[i * 2]);
		data.tolerance[i] = reconstruct_double(&tolerance_bits[i * 2]);
	}
	data.valid = true;
	*out = data;
	return true;
}

void publish_request(const double* targets, const double* tolerance, int max_rpm,
                     int timeout_ms, std::uint32_t* out_id,
                     std::uint32_t* out_epoch) {
	const int published = g_published_buffer.load(std::memory_order_seq_cst);
	const int write_index = (published == 0) ? 1 : 0;
	const std::uint32_t id =
	    g_next_request_id.fetch_add(1, std::memory_order_relaxed) + 1;
	const std::uint32_t epoch = g_cancel_epoch.load(std::memory_order_acquire);

	RequestBuffer& buffer = g_request_buffers[write_index];
	const std::uint32_t w = begin_write(buffer.seq);
	buffer.id.store(id, std::memory_order_seq_cst);
	buffer.epoch.store(epoch, std::memory_order_seq_cst);
	for (int i = 0; i < SMALL_MOTOR_COUNT; ++i) {
		store_double_words(&buffer.target_words[i * 2], targets[i]);
		store_double_words(&buffer.tolerance_words[i * 2], tolerance[i]);
	}
	buffer.max_rpm.store(static_cast<std::uint32_t>(max_rpm),
	                     std::memory_order_seq_cst);
	buffer.timeout_ms.store(static_cast<std::uint32_t>(timeout_ms),
	                        std::memory_order_seq_cst);
	end_write(buffer.seq, w);

	// The request becomes visible only after every field is complete.
	g_published_buffer.store(write_index, std::memory_order_seq_cst);
	*out_id = id;
	*out_epoch = epoch;
}

bool read_ack(std::uint32_t id, StartupStatus* startup, ResultStatus* result) {
	bool found = false;
	StartupStatus chosen_startup = StartupStatus::pending;
	ResultStatus chosen_result = ResultStatus::none;

	for (int i = 0; i < SMALL_MOTOR_ACK_SLOTS; ++i) {
		AckSlot& slot = g_acks[i];
		const std::uint32_t s1 = slot.seq.load(std::memory_order_seq_cst);
		if (s1 & 1u) {
			continue;
		}

		const std::uint32_t ack_id = slot.id.load(std::memory_order_seq_cst);
		const StartupStatus ack_startup = static_cast<StartupStatus>(
		    slot.startup.load(std::memory_order_seq_cst));
		const ResultStatus ack_result = static_cast<ResultStatus>(
		    slot.result.load(std::memory_order_seq_cst));

		const std::uint32_t s2 = slot.seq.load(std::memory_order_seq_cst);
		if (s1 != s2 || ack_id != id) {
			continue;
		}

		if (!found) {
			chosen_startup = ack_startup;
			chosen_result = ack_result;
			found = true;
			continue;
		}

		// A request can have a startup ack and a later completion ack in different
		// slots. Prefer the terminal result over "still running".
		if (chosen_result == ResultStatus::none &&
		    ack_result != ResultStatus::none) {
			chosen_startup = ack_startup;
			chosen_result = ack_result;
		} else if (chosen_result == ResultStatus::none &&
		           ack_result == ResultStatus::none &&
		           chosen_startup == StartupStatus::pending &&
		           ack_startup != StartupStatus::pending) {
			chosen_startup = ack_startup;
		}
	}

	if (!found) {
		return false;
	}
	*startup = chosen_startup;
	*result = chosen_result;
	return true;
}

void publish_ack(std::uint32_t id, StartupStatus startup, ResultStatus result) {
	const std::uint32_t ticket =
	    g_ack_ticket.fetch_add(1, std::memory_order_relaxed);
	AckSlot& slot = g_acks[ticket % SMALL_MOTOR_ACK_SLOTS];
	const std::uint32_t w = begin_write(slot.seq);
	slot.id.store(id, std::memory_order_seq_cst);
	slot.startup.store(static_cast<std::uint32_t>(startup),
	                   std::memory_order_seq_cst);
	slot.result.store(static_cast<std::uint32_t>(result),
	                  std::memory_order_seq_cst);
	end_write(slot.seq, w);
}

bool newer_request_seen(std::uint32_t id) {
	RequestData published;
	return read_published_request(&published) && id_newer(published.id, id);
}

// Convert motor-shaft rotations to one motor's encoder units. Degrees and
// rotations are exact. Counts is deliberately unsupported: the installed PROS
// 4.2.1 headers describe it only as "raw encoder ticks" and do not state how
// many counts make one motor-shaft rotation for these 5.5W motors, so using a
// guessed factor would silently make the public units wrong.
bool rotations_to_units(double rotations, pros::MotorUnits units, double* out) {
	switch (units) {
		case pros::MotorUnits::degrees:
			*out = rotations * 360.0;
			return true;
		case pros::MotorUnits::rotations:
			*out = rotations;
			return true;
		default:
			return false;
	}
}

const char* units_name(pros::MotorUnits units) {
	switch (units) {
		case pros::MotorUnits::degrees:
			return "degrees";
		case pros::MotorUnits::rotations:
			return "rotations";
		case pros::MotorUnits::counts:
			return "counts";
		default:
			return "invalid/unknown";
	}
}

bool validate_common(int max_rpm, int timeout_ms) {
	if (max_rpm <= 0 || max_rpm > SMALL_MOTOR_MAX_RPM) {
		printf("[small_motor] max_rpm must be 1..%d for the green 5.5W motors, got %d\n",
		       SMALL_MOTOR_MAX_RPM, max_rpm);
		return false;
	}
	if (timeout_ms <= 0) {
		printf("[small_motor] timeout_ms must be positive, got %d\n", timeout_ms);
		return false;
	}
	return true;
}

bool read_pair(pros::Motor (&motors)[SMALL_MOTOR_COUNT], double* positions,
               double* units_per_rotation) {
	for (int i = 0; i < SMALL_MOTOR_COUNT; ++i) {
		const pros::MotorUnits units = motors[i].get_encoder_units();
		if (!rotations_to_units(1.0, units, &units_per_rotation[i])) {
			if (units == pros::MotorUnits::counts) {
				printf("[small_motor] motor %d uses COUNTS; PROS 4.2.1 does not "
				       "document counts per motor-shaft rotation for this 5.5W motor. "
				       "Use degrees or rotations, or add a measured counts/rev constant.\n",
				       i);
			} else {
				printf("[small_motor] motor %d has unusable encoder units (%s)\n", i,
				       units_name(units));
			}
			return false;
		}

		const double position = motors[i].get_position();
		if (!std::isfinite(position)) {
			printf("[small_motor] motor %d position read failed\n", i);
			return false;
		}
		positions[i] = position;
	}
	return true;
}

bool competition_disabled() {
	// Only treat this as a cancellation when field control is actually
	// connected, so a bench test without a competition switch can still run.
	return pros::competition::is_connected() && pros::competition::is_disabled();
}

void clear_active() {
	g_active = false;
	g_active_id = 0;
	g_active_epoch = 0;
	g_active_start_ms = 0;
	g_active_timeout_ms = 0;
	g_active_within = false;
	g_active_within_since = 0;
	for (int i = 0; i < SMALL_MOTOR_COUNT; ++i) {
		g_active_targets[i] = 0.0;
		g_active_tolerance[i] = 0.0;
	}
}

void stop_both() {
	// brake() sets velocity to zero for every motor in the group and uses the
	// brake mode already configured on the motors; it does not change that mode.
	small_motor.brake();
}

// Publish a failure for the active request and stop both motors. The worker is
// the only task that issues motor commands or brakes, so this is the handoff.
void fail_active(ResultStatus result) {
	if (!g_active) {
		stop_both();
		return;
	}
	publish_ack(g_active_id, StartupStatus::accepted, result);
	stop_both();
	clear_active();
}

void cancel_active() {
	if (g_active) {
		publish_ack(g_active_id, StartupStatus::accepted, ResultStatus::cancelled);
	}
	stop_both();
	clear_active();
}

// Start one request. Commands are issued once, by the worker, under the
// deadline/disable/cancellation checks. The startup acknowledgement is
// published only after both commands were accepted.
void start_move(const RequestData& request) {
	if (g_cancel_epoch.load(std::memory_order_acquire) != request.epoch) {
		publish_ack(request.id, StartupStatus::rejected, ResultStatus::cancelled);
		return;
	}
	if (competition_disabled()) {
		publish_ack(request.id, StartupStatus::rejected, ResultStatus::failure);
		stop_both();
		return;
	}

	const std::uint32_t start = pros::millis();

	const std::vector<std::int8_t> ports = small_motor.get_port_all();
	if (ports.size() != SMALL_MOTOR_COUNT) {
		printf("[small_motor] expected %d motors, group reports %d\n",
		       SMALL_MOTOR_COUNT, static_cast<int>(ports.size()));
		publish_ack(request.id, StartupStatus::rejected, ResultStatus::failure);
		stop_both();
		return;
	}

	// Motor(port) leaves gearset and encoder units untouched (its defaults are
	// invalid), so this preserves the group's existing configuration. The signed
	// port keeps the group's reversal convention.
	pros::Motor motors[SMALL_MOTOR_COUNT] = {
	    pros::Motor(ports[0]), pros::Motor(ports[1])};

	for (int i = 0; i < SMALL_MOTOR_COUNT; ++i) {
		if (g_cancel_epoch.load(std::memory_order_acquire) != request.epoch) {
			publish_ack(request.id, StartupStatus::rejected,
			            ResultStatus::cancelled);
			stop_both();
			return;
		}
		if (competition_disabled()) {
			publish_ack(request.id, StartupStatus::rejected,
			            ResultStatus::failure);
			stop_both();
			return;
		}
		if ((pros::millis() - start) >=
		    static_cast<std::uint32_t>(request.timeout_ms)) {
			publish_ack(request.id, StartupStatus::rejected,
			            ResultStatus::failure);
			stop_both();
			return;
		}

		if (newer_request_seen(request.id)) {
			// A newer fully published request owns the motors now. Do not stop
			// it; it will issue its own commands on the next worker iteration.
			publish_ack(request.id, StartupStatus::rejected,
			            ResultStatus::replaced);
			return;
		}

		const std::int32_t result =
		    motors[i].move_absolute(request.targets[i], request.max_rpm);
		if (result == PROS_ERR) {
			printf("[small_motor] motor %d rejected the position command\n", i);
			publish_ack(request.id, StartupStatus::rejected,
			            ResultStatus::failure);
			stop_both();
			return;
		}
	}

	// Both commands were issued, but startup is not accepted until this final
	// recheck. If the move expired, was cancelled, or became disabled while motor
	// two was being commanded, stop it now instead of reporting async startup.
	// Stop before publishing the rejection so a false result means the motors
	// have already been handed back.
	if (g_cancel_epoch.load(std::memory_order_acquire) != request.epoch) {
		stop_both();
		publish_ack(request.id, StartupStatus::rejected,
		            ResultStatus::cancelled);
		return;
	}
	if (competition_disabled()) {
		stop_both();
		publish_ack(request.id, StartupStatus::rejected,
		            ResultStatus::failure);
		return;
	}
	if ((pros::millis() - start) >=
	    static_cast<std::uint32_t>(request.timeout_ms)) {
		stop_both();
		publish_ack(request.id, StartupStatus::rejected,
		            ResultStatus::failure);
		return;
	}

	g_active = true;
	g_active_id = request.id;
	g_active_epoch = request.epoch;
	g_active_start_ms = start;
	g_active_timeout_ms = request.timeout_ms;
	g_active_within = false;
	g_active_within_since = 0;
	for (int i = 0; i < SMALL_MOTOR_COUNT; ++i) {
		g_active_targets[i] = request.targets[i];
		g_active_tolerance[i] = request.tolerance[i];
	}

	// Both commands are accepted. async callers return true here; blocking
	// callers keep waiting for success/failure/replacement/cancellation.
	publish_ack(request.id, StartupStatus::accepted, ResultStatus::none);
}

void monitor_active() {
	if (!g_active) {
		return;
	}

	if (g_cancel_epoch.load(std::memory_order_acquire) != g_active_epoch) {
		cancel_active();
		return;
	}

	if (newer_request_seen(g_active_id)) {
		// A newer move replaces this one. Do not brake; the new move commands
		// the motors on the next worker iteration.
		publish_ack(g_active_id, StartupStatus::accepted, ResultStatus::replaced);
		clear_active();
		return;
	}

	double positions[SMALL_MOTOR_COUNT];
	bool read_ok = true;
	for (int i = 0; i < SMALL_MOTOR_COUNT; ++i) {
		positions[i] = small_motor.get_position(i);
		if (!std::isfinite(positions[i])) {
			read_ok = false;
		}
	}

	const std::uint32_t now = pros::millis();

	// Cancellation, newer move, disable and the deadline are all checked before
	// success can be reported.
	if (g_cancel_epoch.load(std::memory_order_acquire) != g_active_epoch) {
		cancel_active();
		return;
	}
	if (newer_request_seen(g_active_id)) {
		publish_ack(g_active_id, StartupStatus::accepted, ResultStatus::replaced);
		clear_active();
		return;
	}
	if (competition_disabled()) {
		fail_active(ResultStatus::failure);
		return;
	}
	if ((now - g_active_start_ms) >=
	    static_cast<std::uint32_t>(g_active_timeout_ms)) {
		fail_active(ResultStatus::failure);
		return;
	}
	if (!read_ok) {
		fail_active(ResultStatus::failure);
		return;
	}

	bool both_within = true;
	for (int i = 0; i < SMALL_MOTOR_COUNT; ++i) {
		if (std::fabs(positions[i] - g_active_targets[i]) >
		    g_active_tolerance[i]) {
			both_within = false;
		}
	}

	if (both_within) {
		if (!g_active_within) {
			g_active_within = true;
			g_active_within_since = now;
		} else if ((now - g_active_within_since) >=
		           static_cast<std::uint32_t>(SMALL_MOTOR_SETTLE_MS)) {
			// Success ends monitoring but leaves the built-in position target
			// active, exactly as before.
			publish_ack(g_active_id, StartupStatus::accepted,
			            ResultStatus::success);
			clear_active();
		}
	} else {
		g_active_within = false;
	}
}

void small_motor_worker() {
	g_worker_ready.store(true, std::memory_order_release);

	while (true) {
		const std::uint32_t cancel = g_cancel_epoch.load(std::memory_order_acquire);

		// Cancellation/stop is handled before any request can command. The
		// worker publishes the stop acknowledgement only after braking, which is
		// the actual handoff to raw driver control.
		if (cancel != g_last_stop_ack) {
			cancel_active();
			g_stop_ack_epoch.store(cancel, std::memory_order_release);
			g_last_stop_ack = cancel;
			pros::delay(SMALL_MOTOR_POLL_MS);
			continue;
		}

		RequestData newest;
		const bool have_request = read_published_request(&newest);

		if (g_active && have_request && id_newer(newest.id, g_active_id)) {
			publish_ack(g_active_id, StartupStatus::accepted,
			            ResultStatus::replaced);
			clear_active();
		}

		if (g_active) {
			monitor_active();
			pros::delay(SMALL_MOTOR_POLL_MS);
			continue;
		}

		if (have_request && id_newer(newest.id, g_last_processed_id)) {
			g_last_processed_id = newest.id;
			if (newest.epoch != g_cancel_epoch.load(std::memory_order_acquire)) {
				publish_ack(newest.id, StartupStatus::rejected,
				            ResultStatus::cancelled);
			} else {
				start_move(newest);
			}
		}

		pros::delay(SMALL_MOTOR_POLL_MS);
	}
}

bool wait_for_request(std::uint32_t id, std::uint32_t epoch, bool async) {
	while (true) {
		StartupStatus startup;
		ResultStatus result;
		if (read_ack(id, &startup, &result)) {
			if (startup == StartupStatus::rejected) {
				return false;
			}
			if (startup == StartupStatus::accepted) {
				if (async) {
					return true;
				}
				if (result == ResultStatus::success) {
					// A blocking move that has since been replaced or cancelled
					// returns false even if it completed just before.
					if (g_cancel_epoch.load(std::memory_order_acquire) != epoch) {
						return false;
					}
					if (newer_request_seen(id)) {
						return false;
					}
					return true;
				}
				if (result == ResultStatus::failure ||
				    result == ResultStatus::replaced ||
				    result == ResultStatus::cancelled) {
					return false;
				}
			}
		}

		if (g_cancel_epoch.load(std::memory_order_acquire) != epoch) {
			return false;
		}
		if (newer_request_seen(id)) {
			return false;
		}
		if (!g_worker_ready.load(std::memory_order_acquire)) {
			return false;
		}

		pros::delay(SMALL_MOTOR_WAIT_MS);
	}
}

}  // namespace

void small_motor_init() {
	// Called once from initialize(), which is not deleted by competition mode
	// changes. A task is never created by a move or stop call.
	bool expected = false;
	if (g_worker_created.compare_exchange_strong(
	        expected, true, std::memory_order_acq_rel)) {
		pros::Task worker(
		    []() {
			    small_motor_worker();
		    },
		    TASK_PRIORITY_DEFAULT, 4096, "small_motor");
	}
}

void small_motor_stop() {
	// No mutex is taken here. If this competition task is deleted while waiting,
	// the worker still observes g_cancel_epoch and performs the stop.
	if (!g_worker_ready.load(std::memory_order_acquire)) {
		// The worker cannot be monitoring anything yet. Stop directly; there is
		// no worker write to race with.
		small_motor.brake();
		return;
	}

	const std::uint32_t my_epoch =
	    g_cancel_epoch.fetch_add(1, std::memory_order_acq_rel) + 1;
	while (true) {
		const std::uint32_t ack =
		    g_stop_ack_epoch.load(std::memory_order_acquire);
		if (static_cast<std::int32_t>(ack - my_epoch) >= 0) {
			return;
		}
		if (!g_worker_ready.load(std::memory_order_acquire)) {
			small_motor.brake();
			return;
		}
		pros::delay(SMALL_MOTOR_WAIT_MS);
	}
}

bool small_motor_move_rotations(double rotations, int max_rpm, int timeout_ms,
                                bool async) {
	if (!std::isfinite(rotations)) {
		printf("[small_motor] rotations must be finite\n");
		return false;
	}
	if (!validate_common(max_rpm, timeout_ms)) {
		return false;
	}

	const std::vector<std::int8_t> ports = small_motor.get_port_all();
	if (ports.size() != SMALL_MOTOR_COUNT) {
		printf("[small_motor] expected %d motors, group reports %d\n", SMALL_MOTOR_COUNT,
		       static_cast<int>(ports.size()));
		small_motor_stop();
		return false;
	}

	// Motor(port) leaves gearset and encoder units untouched (its defaults are
	// invalid), so this preserves the group's existing configuration. The signed
	// port keeps the group's reversal convention.
	pros::Motor motors[SMALL_MOTOR_COUNT] = {pros::Motor(ports[0]), pros::Motor(ports[1])};
	double starts[SMALL_MOTOR_COUNT];
	double units_per_rotation[SMALL_MOTOR_COUNT];
	if (!read_pair(motors, starts, units_per_rotation)) {
		small_motor_stop();
		return false;
	}

	// Valid zero relative movement is a non-commanding no-op. It deliberately
	// does not replace or cancel a move that is already active.
	if (rotations == 0.0) {
		return true;
	}

	double targets[SMALL_MOTOR_COUNT];
	double tolerance[SMALL_MOTOR_COUNT];
	for (int i = 0; i < SMALL_MOTOR_COUNT; ++i) {
		targets[i] = starts[i] + rotations * units_per_rotation[i];
		tolerance[i] =
		    SMALL_MOTOR_POSITION_TOLERANCE_ROTATIONS * units_per_rotation[i];
		if (!std::isfinite(targets[i])) {
			printf("[small_motor] relative target for motor %d is not finite\n", i);
			return false;
		}
	}

	if (!g_worker_ready.load(std::memory_order_acquire)) {
		printf("[small_motor] worker is not ready; call small_motor_init() from initialize()\n");
		return false;
	}

	std::uint32_t id = 0;
	std::uint32_t epoch = 0;
	publish_request(targets, tolerance, max_rpm, timeout_ms, &id, &epoch);
	return wait_for_request(id, epoch, async);
}

bool small_motor_move_to(double position_rotations, int max_rpm, int timeout_ms,
                         bool async) {
	if (!std::isfinite(position_rotations)) {
		printf("[small_motor] position_rotations must be finite\n");
		return false;
	}
	if (!validate_common(max_rpm, timeout_ms)) {
		return false;
	}

	const std::vector<std::int8_t> ports = small_motor.get_port_all();
	if (ports.size() != SMALL_MOTOR_COUNT) {
		printf("[small_motor] expected %d motors, group reports %d\n", SMALL_MOTOR_COUNT,
		       static_cast<int>(ports.size()));
		small_motor_stop();
		return false;
	}

	// Motor(port) leaves gearset and encoder units untouched (its defaults are
	// invalid), so this preserves the group's existing configuration. The signed
	// port keeps the group's reversal convention.
	pros::Motor motors[SMALL_MOTOR_COUNT] = {pros::Motor(ports[0]), pros::Motor(ports[1])};
	double starts[SMALL_MOTOR_COUNT];
	double units_per_rotation[SMALL_MOTOR_COUNT];
	if (!read_pair(motors, starts, units_per_rotation)) {
		small_motor_stop();
		return false;
	}

	double targets[SMALL_MOTOR_COUNT];
	double tolerance[SMALL_MOTOR_COUNT];
	for (int i = 0; i < SMALL_MOTOR_COUNT; ++i) {
		targets[i] = position_rotations * units_per_rotation[i];
		tolerance[i] =
		    SMALL_MOTOR_POSITION_TOLERANCE_ROTATIONS * units_per_rotation[i];
		if (!std::isfinite(targets[i])) {
			printf("[small_motor] absolute target for motor %d is not finite\n", i);
			return false;
		}
	}

	if (!g_worker_ready.load(std::memory_order_acquire)) {
		printf("[small_motor] worker is not ready; call small_motor_init() from initialize()\n");
		return false;
	}

	std::uint32_t id = 0;
	std::uint32_t epoch = 0;
	publish_request(targets, tolerance, max_rpm, timeout_ms, &id, &epoch);
	return wait_for_request(id, epoch, async);
}
