#pragma once

#include <godot_cpp/classes/os.hpp>
#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

namespace halyard {

/// Calculate the full submerged depth based on probe extents in local space
/// This represents the depth at which all probes would be fully submerged
/// Returns a negative value (depth below surface)
inline float calculate_full_submerged_depth(const godot::PackedVector3Array &probes, const Transform3D &global_xform) {
	if (probes.size() == 0) {
		return 0.0f;
	}

	float min_probe_depth = INFINITY;
	float max_probe_depth = -INFINITY;

	for (int i = 0; i < probes.size(); i++) {
		const godot::Vector3 &probe = probes[i];
		Vector3 global_probe = global_xform.xform(probe);
		if (global_probe.y < min_probe_depth) {
			min_probe_depth = global_probe.y;
		}
		if (global_probe.y > max_probe_depth) {
			max_probe_depth = global_probe.y;
		}
	}

	// Return negative value representing depth below surface
	float depth = -abs(max_probe_depth - min_probe_depth);
	if (Math::is_zero_approx(depth)) {
		depth = -0.5f; // Single point? Pretend we're a 1 meter cube
	}

	return depth;
}

#define PERFORMANCE_TESTING
#ifdef PERFORMANCE_TESTING

struct TimerStats {
	uint64_t call_count = 0;
	uint64_t total_usec = 0;

	inline void add_sample(uint64_t elapsed_usec) {
		call_count++;
		total_usec += elapsed_usec;
	}

	inline double get_average_usec() const {
		return call_count > 0 ? (double)total_usec / call_count : 0.0;
	}

	inline void reset() {
		call_count = 0;
		total_usec = 0;
	}
};

// Logs elapsed time when leaving scope
class ScopedTimer {
private:
	const char *_name;
	TimerStats *_stats;
	uint64_t _start_usec;

public:
	// Prevent copying
	ScopedTimer(const ScopedTimer &) = delete;
	ScopedTimer &operator=(const ScopedTimer &) = delete;

	inline ScopedTimer(const char *name, TimerStats *stats) :
			_name(name), _stats(stats) {
		_start_usec = Time::get_singleton()->get_ticks_usec();
	}

	inline ~ScopedTimer() {
		uint64_t end_usec = Time::get_singleton()->get_ticks_usec();
		uint64_t elapsed = end_usec - _start_usec;
		_stats->add_sample(elapsed);

		// Only print every 1000 calls
		if (_stats->call_count % 1000 == 0) {
			double avg = _stats->get_average_usec();
			UtilityFunctions::print(_name, ": ", avg, " usec over ", _stats->call_count, " calls");
			_stats->reset();
		}
	}
};
#define SCOPED_TIMER(name)                           \
	static halyard::TimerStats __timer_stats_##name; \
	halyard::ScopedTimer __timer_##name(#name, &__timer_stats_##name)
#else
#define SCOPED_TIMER(name) \
	{}
#endif

} // namespace halyard