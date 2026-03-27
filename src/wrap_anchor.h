/* Copyright (c) M. Estee. MIT License.

WrapAnchor is a RopeAnchor subclass that generates particle positions by sweeping
a helical path around the local +X axis at a given radius, then ray-casting radially
inward at each step to find the actual surface contact points on nearby geometry.

Consecutive collision points closer than the particle spacing are de-duplicated,
keeping the reported count consistent with the rope's target density.

*/

#pragma once

#include "rope_anchor.h"
#include <godot_cpp/variant/packed_vector3_array.hpp>

using namespace godot;

// An anchor that wraps around geometry along the +X axis, finding surface
// contact points via ray-casting and de-duplicating nearby results.
class WrapAnchor : public RopeAnchor {
	GDCLASS(WrapAnchor, RopeAnchor)

private:
	float _wrap_radius = 1.0f; // sweep radius from the X axis
	float _wrap_length = 5.0f; // total arc length of the helix in meters
	float _max_turns = 10.0f; // maximum number of helix turns
	float _rope_width = 0.125f; // rope width; controls axial pitch per turn
	float _particles_per_meter = 4.0f; // particle density along the arc
	float _unevenness = 0.0f; // 0=even pitch, 1=wobble the wrapping
	float _turn_gap = 0.0f; // extra axial gap inserted between each full turn
	bool _clockwise = false; // winding direction around +X
	float _offset_angle = 0.0f; // starting angle offset in radians
	int _collision_mask = 1; // physics layers to ray-cast against
	bool _debug = false; // draw debug visualisation each frame

	mutable PackedVector3Array _positions; // cached local-space positions
	mutable bool _dirty = true;

	// Rebuilds _positions from current properties via ray-casting.
	void _rebuild_positions() const;

	// Draws debug geometry for the current cached positions and sweep rays.
	void _draw_debug();

	// Draws an editor gizmo circle in the YZ plane showing the wrap_radius.
	void _draw_editor_gizmo();

protected:
	static void _bind_methods();
	void _notify_anchor_changed() override;
	void _notification(int p_what);

public:
	WrapAnchor() = default;
	virtual ~WrapAnchor() override = default;

	PROPERTY_GET_SET(float, wrap_radius, { _notify_anchor_changed(); })
	PROPERTY_GET_SET(float, wrap_length, { _notify_anchor_changed(); })
	PROPERTY_GET_SET(float, max_turns, { _notify_anchor_changed(); })
	PROPERTY_GET_SET(float, rope_width, { _notify_anchor_changed(); })
	PROPERTY_GET_SET(float, particles_per_meter, { _notify_anchor_changed(); })
	PROPERTY_GET_SET(float, unevenness, { _notify_anchor_changed(); })
	PROPERTY_GET_SET(float, turn_gap, { _notify_anchor_changed(); })
	PROPERTY_GET_SET(bool, clockwise, { _notify_anchor_changed(); })
	PROPERTY_GET_SET(float, offset_angle, { _notify_anchor_changed(); })
	PROPERTY_GET_SET(int, collision_mask, { _notify_anchor_changed(); })

	// debug has a custom setter so it can toggle process on/off.
	PROPERTY_GET(bool, debug)
	void set_debug(bool p_val);

	int get_particle_count() const override;
	Vector3 get_particle_position(int p_idx = 0) const override;
};
