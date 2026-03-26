/* Copyright (c) M. Estee. MIT License. */

#include "wrap_anchor.h"

#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/physics_direct_space_state3d.hpp>
#include <godot_cpp/classes/physics_ray_query_parameters3d.hpp>
#include <godot_cpp/classes/physics_server3d.hpp>
#include <godot_cpp/classes/world3d.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/variant/transform3d.hpp>

#include "dd3d_cpp_api.hpp"

void WrapAnchor::_bind_methods() {
	EXPORT_PROPERTY_RANGED(Variant::FLOAT, wrap_radius, WrapAnchor, "0.01,100,0.001");
	EXPORT_PROPERTY_RANGED(Variant::FLOAT, wrap_length, WrapAnchor, "0.01,200,0.01");
	EXPORT_PROPERTY_RANGED(Variant::FLOAT, max_turns, WrapAnchor, "0.1,100,0.1");
	EXPORT_PROPERTY_RANGED(Variant::FLOAT, rope_width, WrapAnchor, "0.001,10,0.001");
	EXPORT_PROPERTY_RANGED(Variant::FLOAT, particles_per_meter, WrapAnchor, "0.1,32,0.1");
	EXPORT_PROPERTY_RANGED(Variant::FLOAT, unevenness, WrapAnchor, "0.0,1.0,0.01");
	EXPORT_PROPERTY_RANGED(Variant::FLOAT, turn_gap, WrapAnchor, "-1.0,1.0,0.001");
	EXPORT_PROPERTY(Variant::BOOL, clockwise, WrapAnchor);
	EXPORT_PROPERTY_RANGED(Variant::FLOAT, offset_angle, WrapAnchor, "-360,360,0.1,radians_as_degrees");

	ClassDB::bind_method(D_METHOD("set_collision_mask", "mask"), &WrapAnchor::set_collision_mask);
	ClassDB::bind_method(D_METHOD("get_collision_mask"), &WrapAnchor::get_collision_mask);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_mask", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_mask", "get_collision_mask");

	EXPORT_PROPERTY(Variant::BOOL, debug, WrapAnchor);
}

#pragma region Notifications

void WrapAnchor::_notify_anchor_changed() {
	_dirty = true;
	RopeAnchor::_notify_anchor_changed();
}

void WrapAnchor::set_debug(bool p_val) {
	if (p_val != _debug) {
		_debug = p_val;
		// only drive process when debug is on and we are inside the tree
		set_process(_debug && is_inside_tree());
		_notify_anchor_changed();
	}
}

void WrapAnchor::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {
			set_notify_transform(true);
			// always process in the editor so the radius gizmo stays visible
			if (_debug || Engine::get_singleton()->is_editor_hint()) {
				set_process(true);
			}
			_dirty = true;
		} break;

		case NOTIFICATION_EXIT_TREE: {
			set_process(false);
		} break;

		case NOTIFICATION_TRANSFORM_CHANGED: {
			// global transform changed, so world-space ray results are stale
			_dirty = true;
		} break;

		case NOTIFICATION_PROCESS: {
			if (_debug || Engine::get_singleton()->is_editor_hint()) {
				auto config = DebugDraw3D::new_scoped_config();
				config->set_thickness(0.005);

				if (_debug) {
					_draw_debug();
				}
				if (Engine::get_singleton()->is_editor_hint()) {
					_draw_editor_gizmo();
				}
			}
		} break;

		default:
			break;
	}
}

#pragma endregion

#pragma region Position Rebuild

// Sweeps a helical path around the local +X axis, ray-casting radially inward
// at each step to find surface contact points.  Results are stored in local space.
// Wrapping stops when the accumulated real arc distance reaches _wrap_length
// or the angular sweep reaches _max_turns, whichever comes first.
void WrapAnchor::_rebuild_positions() const {
	_positions.clear();

	if (!is_inside_tree() || _collision_mask == 0) {
		return;
	}

	Ref<World3D> world = get_world_3d();
	if (world.is_null()) {
		return;
	}

	PhysicsDirectSpaceState3D *dss = PhysicsServer3D::get_singleton()->space_get_direct_state(world->get_space());
	if (dss == nullptr) {
		return;
	}

	// maximum theoretical particle count, will likely exit early due to the _wrap_length limit
	float circumference = Math_TAU * _wrap_radius;
	int count = MAX(2, static_cast<int>(Math::round(_max_turns * circumference * _particles_per_meter)));
	float min_spacing = 1.0f / _particles_per_meter;

	float winding_dir = _clockwise ? -1.0f : 1.0f;

	Transform3D global_xform = get_global_transform();
	Transform3D global_xform_inv = global_xform.inverse();

	Ref<PhysicsRayQueryParameters3D> ray_query;
	ray_query.instantiate();
	ray_query->set_collision_mask(_collision_mask);
	ray_query->set_collide_with_areas(false);
	ray_query->set_collide_with_bodies(true);
	ray_query->set_hit_from_inside(false);

	// track accumulated real surface distance to enforce _wrap_length budget
	float accumulated_length = 0.0f;
	Vector3 prev_local;
	bool has_prev = false;

	for (int idx = 0; idx < count; idx++) {
		float t = static_cast<float>(idx) / static_cast<float>(count);
		float angle = t * _max_turns * Math_TAU * winding_dir + _offset_angle;

		// even pitch advances linearly, uneven is a sine wave
		float x_even = t * _max_turns * (_rope_width + _turn_gap);
		float phase = 2.0f * t - 1.0f;
		float x_uneven = sin(phase * Math_PI) * x_even;
		float x = Math::lerp(x_even, x_uneven, _unevenness);

		float cy = Math::cos(angle);
		float cz = Math::sin(angle);

		Vector3 local_helix(x, _wrap_radius * cy, _wrap_radius * cz);
		Vector3 local_pos;

		if (_collision_mask != 0) {
			// cast from the helix point inward toward the X axis
			Vector3 local_origin(x, _wrap_radius * cy, _wrap_radius * cz);
			Vector3 local_target(x, 0.0f, 0.0f);

			Vector3 from = global_xform.xform(local_origin);
			Vector3 to = global_xform.xform(local_target);

			ray_query->set_from(from);
			ray_query->set_to(to);

			Dictionary result = dss->intersect_ray(ray_query);
			if (!result.is_empty()) {
				Vector3 normal = result["normal"];
				Vector3 contact = result["position"];

				// move contact back by the rope radius
				contact += normal.normalized() * _rope_width / 2.0;

				// convert the world-space hit back to local space
				local_pos = global_xform_inv.xform(contact);
				if (_debug) {
					// draw the ray for debug purposes
					DebugDraw3D::draw_square(result["position"], 0.01, Color(1, 0, 1), .1);
				}
			} else {
				// no hit: skip this step
				continue;
			}
		}

		// de-duplicate: skip points that are too close to the previous accepted point
		if (has_prev && local_pos.distance_to(prev_local) < min_spacing) {
			continue;
		}

		// accumulate real arc distance and stop once the wrap_length budget is spent
		if (has_prev) {
			accumulated_length += local_pos.distance_to(prev_local);
			if (accumulated_length >= _wrap_length) {
				_positions.push_back(local_pos);
				break;
			}
		}

		// only add collision points.
		_positions.push_back(local_pos);
		prev_local = local_pos;
		has_prev = true;
	}

	_dirty = false;
}

#pragma endregion

#pragma region Debug

void WrapAnchor::_draw_debug() {
	if (_dirty) {
		_rebuild_positions();
	}

	Transform3D global_xform = get_global_transform();
	int n = _positions.size();

	PackedVector3Array pts;
	pts.resize(n);
	for (int idx = 0; idx < n; idx++) {
		Vector3 world_pos = global_xform.xform(_positions[idx]);
		pts.set(idx, world_pos);
		DebugDraw3D::draw_square(world_pos, 0.01f, Color(1, 0.5f, 0, 1));
	}

	if (n > 1) {
		DebugDraw3D::draw_line_path(pts, Color(1, 1, 0, 1));
	}
}

void WrapAnchor::_draw_editor_gizmo() {
	// draw a circle in the YZ plane at the local origin to visualise wrap_radius
	const int segments = 32;
	Transform3D global_xform = get_global_transform();

	PackedVector3Array pts;
	pts.resize(segments + 1);
	for (int idx = 0; idx <= segments; idx++) {
		float angle = (static_cast<float>(idx) / static_cast<float>(segments)) * Math_TAU;
		Vector3 local_pt(0.0f, Math::cos(angle) * _wrap_radius, Math::sin(angle) * _wrap_radius);
		pts.set(idx, global_xform.xform(local_pt));
	}
	DebugDraw3D::draw_line_path(pts, Color(0, 0.8f, 1, 0.8f));
}

#pragma endregion

#pragma region Anchor Overrides

int WrapAnchor::get_particle_count() const {
	if (_dirty) {
		_rebuild_positions();
	}
	return MAX(1, _positions.size());
}

Vector3 WrapAnchor::get_particle_position(int p_idx) const {
	if (_dirty) {
		_rebuild_positions();
	}
	if (p_idx < 0 || p_idx >= _positions.size()) {
		return Vector3();
	}
	return _positions[p_idx];
}

#pragma endregion
