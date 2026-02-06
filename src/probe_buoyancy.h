/* ProbeBuoyancy

A utility class for calculating buoyancy forces on a set of point probes.

This class encapsulates the functionality for probe-based buoyancy calculations,
including depth calculation, force computation, and transform caching.
*/

#pragma once

#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/templates/vector.hpp>

#include "halyard_utils.h"

using namespace godot;

class LiquidArea;

namespace halyard {

class ProbeBuoyancy {
private:
	// Configuration
	PackedVector3Array _probes;
	LiquidArea* _liquid_area = nullptr;
	float _mass = 1.0f;
	float _buoyancy = 1.0f;
	bool _ignore_waves = false;

	// Cached data
	Vector<Transform3D> _last_probe_transforms;
	PackedVector3Array _forces;
	float _full_submerged_depth = 0.0f;
    int _last_submerged_count = 0;

	// Derived properties
	float _character_density = 1000.0f;
	float _character_volume = 0.001f;

	void _update_derived_properties();

public:
	ProbeBuoyancy() = default;

	// Accessors
	void set_probes(const PackedVector3Array& p_probes);
	const PackedVector3Array& get_probes() const;

    // Liquid area the probes are interacting with
	void set_liquid_area(LiquidArea* p_liquid_area);
	LiquidArea* get_liquid_area() const;

    // Mass in kg of the object represented by the probes
	void set_mass(float p_mass);
	float get_mass() const;

    // Buoyancy is a scalar. A value of 1.0 means neutral buoyancy (displaces its own weight in fluid).
    // Density is derived from mass and buoyancy.
	void set_buoyancy(float p_buoyancy);
	float get_buoyancy() const;

    // disable the wave effects on this probe buoyancy calculation
	void set_ignore_waves(bool p_ignore);
	bool get_ignore_waves() const;

    // derived properties
	float get_density() const;
	float get_volume() const;

    // the forces for each probe
	const PackedVector3Array& get_forces() const;

	// Get the last calculated transforms for each probe
	const Vector<Transform3D>& get_last_transforms() const;
    int get_last_submerged_count() const;

    // depth the probeset must be at for all probes to be submerged.
	float get_full_submerged_depth() const;

	// Update cached wave transforms for each probe
	// Should be called before update_forces() to cache wave surface data
	void update_transforms(const Transform3D& body_transform);

	// Calculate buoyancy forces for each probe
	// Call update_transforms() first to ensure transforms are up to date
	void update_forces(const Transform3D& body_transform, const Vector3& gravity);

	// Get the number of probes that are currently submerged
	int get_submerged_count(const Transform3D& body_transform) const;
    float get_submerged_ratio() const;
};

} // namespace halyard
