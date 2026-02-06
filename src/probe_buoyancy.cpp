/* ProbeBuoyancy
 *
 * A utility class for calculating buoyancy forces on a set of point probes.
 *
 * This class encapsulates the functionality for probe-based buoyancy calculations,
 * including depth calculation, force computation, and transform caching.
 *
 * Copyright (c) M. Estee
 * MIT License.
 */

#include "probe_buoyancy.h"
#include "liquid_area.h"

using namespace godot;
using namespace halyard;

#pragma region Private Helpers

void ProbeBuoyancy::_update_derived_properties() {
	if (_liquid_area == nullptr) {
		_character_density = 1000.0f;
		_character_volume = _mass / 1000.0f;
		return;
	}

	float fluid_density = _liquid_area->get_density();
	_character_density = fluid_density / _buoyancy;
	_character_volume = _mass / _character_density;
}

#pragma endregion

#pragma region Accessors

void ProbeBuoyancy::set_probes(const PackedVector3Array& p_probes) {
	_probes = p_probes;
	_forces.resize(_probes.size());
	_last_probe_transforms.resize(_probes.size());
}

const PackedVector3Array& ProbeBuoyancy::get_probes() const {
	return _probes;
}

void ProbeBuoyancy::set_liquid_area(LiquidArea* p_liquid_area) {
	_liquid_area = p_liquid_area;
	_update_derived_properties();
}

LiquidArea* ProbeBuoyancy::get_liquid_area() const {
	return _liquid_area;
}

void ProbeBuoyancy::set_mass(float p_mass) {
	_mass = p_mass;
	_update_derived_properties();
}

float ProbeBuoyancy::get_mass() const {
	return _mass;
}

void ProbeBuoyancy::set_buoyancy(float p_buoyancy) {
	_buoyancy = p_buoyancy;
	_update_derived_properties();
}

float ProbeBuoyancy::get_buoyancy() const {
	return _buoyancy;
}

void ProbeBuoyancy::set_ignore_waves(bool p_ignore) {
	_ignore_waves = p_ignore;
}

bool ProbeBuoyancy::get_ignore_waves() const {
	return _ignore_waves;
}

float ProbeBuoyancy::get_density() const {
	return _character_density;
}

float ProbeBuoyancy::get_volume() const {
	return _character_volume;
}

const PackedVector3Array& ProbeBuoyancy::get_forces() const {
	return _forces;
}

const Vector<Transform3D>& ProbeBuoyancy::get_last_transforms() const {
	return _last_probe_transforms;
}

int ProbeBuoyancy::get_last_submerged_count() const {
    return _last_submerged_count;
}

float ProbeBuoyancy::get_submerged_ratio() const {
    if (_probes.size() == 0) {
        return 0.0f;
    }
    return (float)_last_submerged_count / (float)_probes.size();
}  

float ProbeBuoyancy::get_full_submerged_depth() const {
	return _full_submerged_depth;
}

#pragma endregion

#pragma region Core Calculations

void ProbeBuoyancy::update_transforms(const Transform3D& body_transform) {
	if (_liquid_area == nullptr) {
		return;
	}

	const int probe_count = _probes.size();
	if (probe_count == 0) {
		return;
	}

	// Ensure arrays are properly sized
	_last_probe_transforms.resize(probe_count);

	// Cache liquid surface height
	const float liquid_y = _liquid_area->get_global_transform().origin.y;

	// Calculate full submerged depth for normalization
	_full_submerged_depth = calculate_full_submerged_depth(_probes, body_transform);

	// Update each probe's wave transform
    _last_submerged_count = 0;
	for (int idx = 0; idx < probe_count; ++idx) {
		// Get probe position in global space
		Vector3 probe = body_transform.xform(_probes[idx]);

		// Get liquid position (flat tidal coordinate)
		Vector3 liquid_pos = probe;
		liquid_pos.y = liquid_y;

		// Get wave transform at this position
		Transform3D wave_xform = Transform3D(Basis(), liquid_pos);
		if (!_ignore_waves) {
			wave_xform = _liquid_area->get_liquid_transform(liquid_pos);
		}

		// Cache the transform
		_last_probe_transforms.write[idx] = wave_xform;

        // count submerged probes
        float wave_depth = probe.y - wave_xform.origin.y;
        if (wave_depth < 0.0f) {
            _last_submerged_count++;
        }
	}
}

void ProbeBuoyancy::update_forces(const Transform3D& body_transform, const Vector3& gravity) {
	if (_liquid_area == nullptr) {
		// Clear forces if no liquid area
		for (int i = 0; i < _forces.size(); ++i) {
			_forces[i] = Vector3(0, 0, 0);
		}
		return;
	}

	const int probe_count = _probes.size();
	if (probe_count == 0) {
		return;
	}

	// Validate cached transforms
	if (probe_count != _last_probe_transforms.size()) {
		return;
	}

	// Early exit if no depth to submerge
	if (Math::is_zero_approx(_full_submerged_depth)) {
		for (int i = 0; i < _forces.size(); ++i) {
			_forces[i] = Vector3(0, 0, 0);
		}
		return;
	}

	// Pre-calculate constants
	const float probe_ratio = 1.0f / probe_count;
	const float fluid_density = _liquid_area->get_density();

	// Calculate forces for each probe
	for (int idx = 0; idx < probe_count; ++idx) {
		// Get probe position in global space
		Vector3 probe = body_transform.xform(_probes[idx]);

		// Use cached wave transform
		Transform3D wave_xform = _last_probe_transforms[idx];
		Vector3 wave_pos = wave_xform.origin;

		// Calculate wave depth as a ratio between not submerged and fully submerged
		float wave_depth = -(probe.y - wave_pos.y);
		wave_depth = Math::clamp(wave_depth / _full_submerged_depth, -1.0f, 0.0f);

		// Calculate force if submerged
		if (wave_depth < 0.0f) {
			// Each probe affects 1/N of the volume
			float probe_volume = _character_volume * probe_ratio * -wave_depth;

			// Calculate buoyancy force
			Vector3 wave_normal = wave_xform.basis.get_column(1);
			wave_normal = wave_normal.lerp(Vector3(0, 1, 0), probe_ratio).normalized();
			Vector3 wave_force = (wave_normal * -gravity.y) * fluid_density * probe_volume;

			// Add current force
			Vector3 current_force = _liquid_area->get_current_speed() * probe_volume;

			_forces[idx] = wave_force + current_force;
		} else {
			_forces[idx] = Vector3(0, 0, 0);
		}
	}
}

int ProbeBuoyancy::get_submerged_count(const Transform3D& body_transform) const {
	if (_liquid_area == nullptr || _probes.size() == 0) {
		return 0;
	}

	int count = 0;
	const float liquid_y = _liquid_area->get_global_transform().origin.y;

	for (int idx = 0; idx < _probes.size(); ++idx) {
		Vector3 probe = body_transform.xform(_probes[idx]);
		
		if (idx < _last_probe_transforms.size()) {
			const Transform3D& wave_xform = _last_probe_transforms[idx];
			float wave_depth = probe.y - wave_xform.origin.y;
			float liquid_depth = probe.y - liquid_y;

			if (wave_depth < 0.0f || liquid_depth < 0.0f) {
				count++;
			}
		}
	}

	return count;
}

#pragma endregion
