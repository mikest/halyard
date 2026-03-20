/* Copyright (c) M. Estee. MIT License.

CoiledAnchor is a RopeAnchor subclass that generates anchor positions arranged
in a coil/helix pattern. The coil is built from layers of turns wound around a
central axis (local X), with configurable radius, winding direction, and density.

This is the C++ equivalent of the coil geometry from CoiledRope.gd, packaged as
an anchor so any Rope can reference it.

*/

#pragma once

#include "rope_anchor.h"
#include <godot_cpp/variant/packed_vector3_array.hpp>

using namespace godot;

// An anchor that generates positions arranged in a coil/helix pattern.
// Useful for representing rope wound around a drum or winch.
class CoiledAnchor : public RopeAnchor {
	GDCLASS(CoiledAnchor, RopeAnchor)

private:
    float _rope_width = 0.125f;         // width of the rope, used for spacing coils
	float _particles_per_meter = 2.0f;  // particle density along the coil, should match target ropes.
    float _coiled_length = 1.0f;        // total length of rope represented by the coil, used to determine how many turns to generate
	float _radius = 0.25f;              // radius of the coil
    bool _clockwise = false;            // winding direction of the coil
	float _turns_per_layer = 4.0f;      // number of turns for each wound layer.

	// Cached coil positions, rebuilt lazily when properties change.
	mutable PackedVector3Array _positions;
	mutable bool _dirty = true;

	void _rebuild_coil_positions() const;

protected:
	static void _bind_methods();

public:
	CoiledAnchor() = default;
	virtual ~CoiledAnchor() override = default;

    PROPERTY_GET_SET(float, rope_width, { _dirty = true; })
	PROPERTY_GET_SET(float, particles_per_meter, { _dirty = true; })
    PROPERTY_GET_SET(float, coiled_length, { _dirty = true; })
    PROPERTY_GET_SET(float, radius, { _dirty = true; })
	PROPERTY_GET_SET(bool, clockwise, { _dirty = true; })
	PROPERTY_GET_SET(float, turns_per_layer, { _dirty = true; })

	// Returns the number of coil positions generated from the current properties.
	int get_particle_count(int p_idx = 0) const override;

	// Returns the coil position at the given index in local space.
	Vector3 get_particle_position(int p_idx = 0) const override;
};
