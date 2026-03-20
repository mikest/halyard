/* Copyright (c) M. Estee. MIT License. */

#include "coiled_anchor.h"
#include <godot_cpp/variant/transform3d.hpp>

void CoiledAnchor::_bind_methods() {
	EXPORT_PROPERTY_RANGED(Variant::FLOAT, coiled_length, CoiledAnchor, "0,100,0.01");
    EXPORT_PROPERTY_RANGED(Variant::FLOAT, radius, CoiledAnchor, "0.01,10,0.001");
	EXPORT_PROPERTY_RANGED(Variant::FLOAT, turns_per_layer, CoiledAnchor, "0.1,100,0.1");
	EXPORT_PROPERTY_RANGED(Variant::FLOAT, rope_width, CoiledAnchor, "0.01,10,0.01");
	EXPORT_PROPERTY_RANGED(Variant::FLOAT, particles_per_meter, CoiledAnchor, "1,100,0.1");
	EXPORT_PROPERTY(Variant::BOOL, clockwise, CoiledAnchor);
}

#pragma region Coil Generation

// Rebuild the cached coil positions from current properties.
// Ported from CoiledRope.gd _rebuild_coil_positions().
void CoiledAnchor::_rebuild_coil_positions() const {
	_positions.clear();

	int count = static_cast<int>(_coiled_length * _particles_per_meter);
	if (count <= 0) {
		_dirty = false;
		return;
	}

	_positions.resize(count);

	float offset_per_layer = _rope_width * 0.25f;
	float offset_per_turn = _rope_width * 0.6f;

	// accumulated turn count
	float turn = 0.0f;

	for (int idx = 0; idx < count; idx++) {
		// determine which layer we are on
		int layer = static_cast<int>(Math::floor(turn / _turns_per_layer));
		float layer_radius = _radius + offset_per_layer * layer;
		float layer_circumference = Math_TAU * layer_radius;
		float anchors_per_turn = layer_circumference * _particles_per_meter;

		// alternate winding direction per layer
		float turn_dir = (layer % 2 == 0) ? -1.0f : 1.0f;

		// position within the current layer
		float layer_turn = Math::fmod(turn, _turns_per_layer);
		if (layer_turn < 0.0f) {
			layer_turn += _turns_per_layer;
		}

		float turn_x_offset = (turn_dir < 0.0f) ? 0.0f : (-offset_per_turn * _turns_per_layer);
		float x = layer_turn * offset_per_turn * turn_dir + turn_x_offset;
		float y = layer_radius + offset_per_layer * layer;

		// build the transform: translate then rotate around the X axis
		Vector3 axis = Vector3(1, 0, 0);
		Transform3D xform;
		xform = xform.translated_local(Vector3(x, y, 0));
		float angle = turn * Math_TAU * (_clockwise ? -1.0f : 1.0f);
		xform = xform.rotated(axis, angle);

		_positions.set(idx, xform.origin);

		// advance the accumulated turn count
		if (anchors_per_turn > 0.0f) {
			turn += 1.0f / anchors_per_turn;
		}
	}

	_dirty = false;
}

#pragma endregion

#pragma region Anchor Overrides

int CoiledAnchor::get_particle_count(int p_idx) const {
	if (_dirty) {
		_rebuild_coil_positions();
	}
	return MAX(1, _positions.size());
}

Vector3 CoiledAnchor::get_particle_position(int p_idx) const {
	if (_dirty) {
		_rebuild_coil_positions();
	}
	if (p_idx < 0 || p_idx >= _positions.size()) {
		return Vector3();
	}
	return _positions[p_idx];
}

#pragma endregion
