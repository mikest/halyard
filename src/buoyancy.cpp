#include "buoyancy.h"
#include "liquid_area.h"

#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/rigid_body3d.hpp>
#include <godot_cpp/classes/shape3d.hpp>
#include <godot_cpp/classes/convex_polygon_shape3d.hpp>
#include <godot_cpp/classes/sphere_shape3d.hpp>
#include <godot_cpp/classes/box_shape3d.hpp>
#include <godot_cpp/classes/capsule_shape3d.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/classes/performance.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>

using namespace godot;

Buoyancy::Buoyancy() {
}


Buoyancy::~Buoyancy() {
	if (_buoyancy_mesh.is_valid()) {
		_buoyancy_mesh.unref();
	}

	_destroy_debug_mesh();
}

void Buoyancy::_update_configuration_warnings() {
	if (Engine::get_singleton()->is_editor_hint()){
		update_configuration_warnings();
	}
}


PackedStringArray Buoyancy::_get_configuration_warnings() const {
	PackedStringArray what;

	RigidBody3D *body = Object::cast_to<RigidBody3D>(get_parent());
	if (!body && _apply_forces) {
		what.append("Buoyancy must be a child of a RigidBody3D for forces to be applied.");
	}
	
	if (_buoyancy_mode == BUOYANCY_COLLIDER) {
		if (_collider == nullptr) {
			what.append("Missing collider.");
		} else {
			Ref<Shape3D> shape = _collider->get_shape();
			if (!shape.is_valid()) {
				what.append("Missing collider shape.");
			}else{
				ConvexPolygonShape3D *convex_shape = Object::cast_to<ConvexPolygonShape3D>(*shape);
				BoxShape3D *box_shape = Object::cast_to<BoxShape3D>(*shape);
				SphereShape3D *sphere_shape = Object::cast_to<SphereShape3D>(*shape);
				CapsuleShape3D *capsule_shape = Object::cast_to<CapsuleShape3D>(*shape);
				if (!convex_shape && !box_shape && !sphere_shape && !capsule_shape) {
					what.append("Collider shape must be a ConvexPolygonShape3D, BoxShape3D, SphereShape3D, or CapsuleShape3D. Other shapes are not supported.");
				}
			}
		}
	} else{
		if (_buoyancy_probes.size() == 0) {
			what.append("No buoyancy probes defined for point-based buoyancy.");
		}
	}

	// if (!_liquid_area) {
	// 	what.append("Missing LiquidArea. First LiquidArea in the scene tree will be used,\n or liquid level will be Vector3.ZERO");
	// }

	return what;
}


void Buoyancy::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_READY: {
			if (Engine::get_singleton()->is_editor_hint()) {
				// Connect to inspector property edited signal if in editor
				// Note: This might need adjustment based on Godot's API
			}
			_set_dirty();

			set_process_internal(true);
			set_physics_process_internal(true);
		} break;

		case NOTIFICATION_ENTER_TREE: {
			if (Engine::get_singleton()->is_editor_hint() == false) {
				if (_liquid_area == nullptr) {
					SceneTree *tree = get_tree();
					_liquid_area = LiquidArea::get_liquid_area(tree);
				}
			}
		} break;

		case NOTIFICATION_EXIT_TREE: {
		 	_liquid_area = nullptr;
		} break;

		case NOTIFICATION_INTERNAL_PROCESS:{
			if (_dirty && is_node_ready()) {
				_update_statics();
			}

			// lazy add/remove the debug meshes
			if (_show_debug) {
				if (!_debug_mesh_instance) {
					_create_debug_mesh();
				}
				if (_debug_mesh_dirty) {
					_update_debug_mesh();
					_debug_mesh_dirty = false;
				}
			
			} else if (_debug_mesh_instance) {
				_destroy_debug_mesh();
			}
		} break;

		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
			if (!Engine::get_singleton()->is_editor_hint() && is_node_ready()) {
				RigidBody3D *body = Object::cast_to<RigidBody3D>(get_parent());
				if (body && !body->is_freeze_enabled()) {

					uint64_t time = Time::get_singleton()->get_ticks_usec();

					// optionally apply them
					if (_apply_forces){
						float delta = get_physics_process_delta_time();
						if (_buoyancy_mode == BUOYANCY_PROBES) {
							apply_buoyancy_probe_forces(body, delta);
						} else {
							// recalculate buoyancy volumes and centroids
							_update_dynamics();
							apply_buoyancy_mesh_forces(body, delta);
						}
					}

					// Check if submerged changed and emit signal. We only track entering/exiting water as the ratio can change on every frame.
					float current_ratio = get_submerged_ratio();
					if (Math::is_zero_approx(current_ratio) != Math::is_zero_approx(_last_submerged_ratio)) {
						emit_signal("submerged_changed");
					}
					_last_submerged_ratio = current_ratio;

					// update time taken
					uint64_t elapsed = Time::get_singleton()->get_ticks_usec() - time;
					_buoyancy_time = elapsed;
				}
			}
		} break;
	}
}

#pragma region Properties

// Property getters/setters
void Buoyancy::set_liquid_area(LiquidArea *p_area) {
	_liquid_area = p_area;
	_update_configuration_warnings();

	_set_dirty();
	_debug_mesh_dirty = true;
}

LiquidArea* Buoyancy::get_liquid_area() const {
	return _liquid_area;
}

void Buoyancy::set_collider(CollisionShape3D *p_collider) {
	if (_collider != p_collider) {
		_collider = p_collider;
		_set_dirty();

		_update_configuration_warnings();
	}

	_debug_mesh_dirty = true;
}

CollisionShape3D* Buoyancy::get_collider() const {
	return _collider;
}

// Buoyancy mode
void Buoyancy::set_buoyancy_mode(BuoyancyMode p_mode) {
	ERR_FAIL_COND_MSG(p_mode != BUOYANCY_COLLIDER && p_mode != BUOYANCY_PROBES, "Invalid buoyancy mode");
	_buoyancy_mode = p_mode;
	_debug_mesh_dirty = true;
	_set_dirty();
	_update_configuration_warnings();
}

BuoyancyMode Buoyancy::get_buoyancy_mode() const {
	return _buoyancy_mode;
}

// Buoyancy probes
void Buoyancy::set_buoyancy_probes(const PackedVector3Array &p_probes) {
	_buoyancy_probes = p_probes;
	_debug_mesh_dirty = true;
	_set_dirty();
	_update_configuration_warnings();
}

PackedVector3Array Buoyancy::get_buoyancy_probes() const {
	return _buoyancy_probes;
}

void Buoyancy::set_apply_forces(bool p_apply_forces) {
	_apply_forces = p_apply_forces;
	_update_configuration_warnings();
}

float Buoyancy::get_apply_forces() const {
	return _apply_forces;
}

void Buoyancy::set_density(float p_density) {
	_density = p_density;
	_set_dirty();
}

float Buoyancy::get_density() const {
	return _density;
}

void Buoyancy::set_com_offset(Vector3 p_com_offset) {
	_com_offset = p_com_offset;
	_set_dirty();
}

Vector3 Buoyancy::get_com_offset() const {
	return _com_offset;
}

void Buoyancy::set_submerged_linear_drag(float p_drag) {
	_submerged_linear_drag = p_drag;
}

float Buoyancy::get_submerged_linear_drag() const {
	return _submerged_linear_drag;
}

void Buoyancy::set_submerged_angular_drag(float p_drag) {
	_submerged_angular_drag = p_drag;
}

float Buoyancy::get_submerged_angular_drag() const {
	return _submerged_angular_drag;
}

void Buoyancy::set_probe_buoyancy(float p_buoyancy) {
	_probe_buoyancy = p_buoyancy;
}

float Buoyancy::get_probe_buoyancy() const {
	return _probe_buoyancy;
}

void Buoyancy::set_linear_drag_scale(const Vector3 &p_scale) {
	_linear_drag_scale = p_scale;
}

Vector3 Buoyancy::get_linear_drag_scale() const {
	return _linear_drag_scale;
}

void Buoyancy::set_angular_drag_scale(const Vector3 &p_scale) {
	_angular_drag_scale = p_scale;
}

Vector3 Buoyancy::get_angular_drag_scale() const {
	return _angular_drag_scale;
}

void Buoyancy::set_calculate_mass_properties(bool p_calculate) {
	_calculate_mass_properties = p_calculate;
	_set_dirty();
}

bool Buoyancy::get_calculate_mass_properties() const {
	return _calculate_mass_properties;
}

void Buoyancy::set_ignore_waves(bool p_ignore) {
	_ignore_waves = p_ignore;
}

bool Buoyancy::get_ignore_waves() const {
	return _ignore_waves;
}

// Debug
void Buoyancy::set_show_debug(bool p_show) {
	_show_debug = p_show;
	_debug_mesh_dirty = true;
}

bool Buoyancy::get_show_debug() const {
	return _show_debug;
}

void Buoyancy::set_debug_color(const Color &p_color) {
	_debug_color = p_color;
}

Color Buoyancy::get_debug_color() const {
	return _debug_color;
}


// Read only

float Buoyancy::get_submerged_volume() const {
	return _submerged_volume;
}

Vector3 Buoyancy::get_submerged_centroid() const {
	return _submerged_centroid;
}

Vector3 Buoyancy::get_buoyancy_normal() const {
	return _buoyancy_normal;
}

float Buoyancy::get_submerged_ratio() const {
	if (_buoyancy_mode == BUOYANCY_PROBES) {
		// In probe mode, return proportion of submerged probes
		if (_buoyancy_probes.size() == 0) {
			return 0.0f;
		}
		return (float)_submerged_probe_count / (float)_buoyancy_probes.size();
	} else {
		// In collider mode, return volume ratio
		if (Math::is_zero_approx(_mesh_volume)) {
			return 0.0f;
		}
		return _submerged_volume / _mesh_volume;
	}
}

// Statics
float Buoyancy::get_volume() const {
	return _mesh_volume;
}

Vector3 Buoyancy::get_centroid() const {
	return _mesh_centroid;
}


// Info getters
float Buoyancy::get_mass() const {
	RigidBody3D *body = Object::cast_to<RigidBody3D>(get_parent());
	return body ? body->get_mass() : -1.0f;
}

Vector3 Buoyancy::get_center_of_mass() const {
	RigidBody3D *body = Object::cast_to<RigidBody3D>(get_parent());
	return body ? body->get_center_of_mass() : Vector3(0, 0, 0);
}

Vector3 Buoyancy::get_inertia() const {
	RigidBody3D *body = Object::cast_to<RigidBody3D>(get_parent());
	return body ? body->get_inertia() : Vector3(0, 0, 0);
}

float Buoyancy::_get_buoyancy_time() const {
	return _buoyancy_time;
}

#pragma region Mesh Updates

void Buoyancy::_update_statics() {
	if( _collider == nullptr) return;
	if( _buoyancy_mode != BUOYANCY_COLLIDER) return;

	// if this is already a convex shape, use it directly
	Ref<ConvexPolygonShape3D> convex_shape = _collider->get_shape();
	if (convex_shape.is_valid()) {
		_buoyancy_mesh = convex_shape->get_debug_mesh();

	// otherwise, try to create a simplified convex shape from the existing shape's debug mesh
	} else {
		Ref<Shape3D> shape = _collider->get_shape();
		if (shape.is_valid()) {
			Ref<Mesh> mesh = shape->get_debug_mesh();

			// Use simplified convex shape for buoyancy calculations
			Ref<Shape3D> new_shape = mesh->create_convex_shape(true, true);
			if (new_shape.is_valid()) {
				_buoyancy_mesh = Object::cast_to<ConvexPolygonShape3D>(*new_shape)->get_debug_mesh();
			}
		}
	}

	if (_buoyancy_mesh.is_valid() && _buoyancy_mesh->get_surface_count()) {
		_vertex = _buoyancy_mesh->get_faces();

		// Calculate volume and centroid
		_mesh_volume = 0.0f;
		_mesh_centroid = Vector3(0, 0, 0);
		Vector3 o = Vector3(0, 0, 0);

		// integrate tetrahedrons formed by each triangle and the origin
		int face_count = _vertex.size() / 3;
		for (int idx = 0; idx < face_count; ++idx) {
			Vector3 a = _vertex[idx * 3 + 0];
			Vector3 b = _vertex[idx * 3 + 1];
			Vector3 c = _vertex[idx * 3 + 2];

			Vector4 tri = _tri_contribution(a, b, c, o);
			_mesh_centroid += Vector3(tri.x, tri.y, tri.z);
			_mesh_volume += tri.w;
		}

		if (!Math::is_zero_approx(_mesh_volume)) {
			_mesh_centroid /= (_mesh_volume * 4.0f / 3.0f);
			_mesh_centroid = _collider->get_transform().xform(_mesh_centroid);
		} else {
			_mesh_centroid = _collider->get_global_transform().origin;
		}

		// Auto detect negative mass and recalc with sign flipped
		if (_mesh_volume < 0.0f) {
			_sign = -_sign;
			_update_statics();

			// Bail early to avoid double processing of mass and inertia
			return;
		}

		// Update mass from collider
		RigidBody3D *body = Object::cast_to<RigidBody3D>(get_parent());
		if (body && _calculate_mass_properties) {

			float mass = _mesh_volume * _density;
			body->set_mass(mass);
			body->set_center_of_mass_mode(RigidBody3D::CENTER_OF_MASS_MODE_CUSTOM);
			body->set_center_of_mass(_mesh_centroid + _com_offset);

			// Calculate inertia tensor
			float ix = 0.0f, iy = 0.0f, iz = 0.0f;
			AABB aabb = _buoyancy_mesh->get_aabb();
			// Simplified inertia calculation
			ix = (1.0f / 12.0f) * mass * (aabb.size.y * aabb.size.y + aabb.size.z * aabb.size.z);
			iy = (1.0f / 12.0f) * mass * (aabb.size.x * aabb.size.x + aabb.size.z * aabb.size.z);
			iz = (1.0f / 12.0f) * mass * (aabb.size.x * aabb.size.x + aabb.size.y * aabb.size.y);
			body->set_inertia(Vector3(ix, iy, iz));
		}

		// clear flag
		_dirty = false;
	}
}

void Buoyancy::_update_dynamics() {
	if( _collider == nullptr) return;
	if( _buoyancy_mode != BUOYANCY_COLLIDER) return;

	// clear the _submerged verts for debug
	_submerged_verts.clear();
	_debug_mesh_dirty = true;

	// Get the transformed mesh points and their depths
	_depth_map.clear();
	_depth_map.reserve(_vertex.size());
	_depths.resize(_vertex.size());

	// Reset the averaged wave normal
	_buoyancy_normal = Vector3(0, 0, 0);

	// Dedupe the vertecies.
	// For each unique vertex, calculate its depth once and store the transform.
	// Reuse the transform for duplicate vertex locations.
	// This avoids multiple expensive calls out to LiquidArea per unique vertex.
	Transform3D collider_xform = _collider->get_global_transform();
	for (int idx = 0; idx < _vertex.size(); ++idx) {
		Vector3 global_vertex = collider_xform.xform(_vertex[idx]);
		Transform3D xform;	// Identity

		// cache hit, reuse
		if (_depth_map.has(global_vertex)) {
			xform = _depth_map[global_vertex];
			_depths[idx] = global_vertex.y - xform.origin.y;
		} else {
			// cache miss, calculate
			if (_liquid_area){
				if (_ignore_waves) {
					float liquid_y = _liquid_area ? _liquid_area->get_global_transform().origin.y : 0.0f;
					xform = Transform3D(Basis(), Vector3(global_vertex.x, liquid_y, global_vertex.z));
				} else {
					xform = _liquid_area->get_liquid_transform(global_vertex);
				}
			}
			// update
			_depth_map[global_vertex] = xform;
		}

		// set depth y value for this vertex, depths are in local collider space
		_depths[idx] = global_vertex.y - xform.origin.y;

		// average in this normal
		_buoyancy_normal += xform.basis.get_column(1) / (float)_vertex.size();
	}

	_buoyancy_normal = _buoyancy_normal.normalized();

	// Calculate the locations for each face
	int face_count = _vertex.size() / 3;

	Vector3 o = Vector3(0, 0, 0);
	_submerged_centroid = Vector3(0, 0, 0);
	_submerged_volume = 0.0f;

	for (int idx = 0; idx < face_count; ++idx) {
		// Note that we do the volume calculation in local space
		Vector3 a = _vertex[idx * 3 + 0];
		Vector3 b = _vertex[idx * 3 + 1];
		Vector3 c = _vertex[idx * 3 + 2];

		// Update the locations
		float _a = _depths[idx * 3 + 0];
		float _b = _depths[idx * 3 + 1];
		float _c = _depths[idx * 3 + 2];

		// Calculate submerged contribution
		Vector4 submerged_tri = _partial_intersection(a, b, c, o, _a, _b, _c, true);
		_submerged_centroid += Vector3(submerged_tri.x, submerged_tri.y, submerged_tri.z);
		_submerged_volume += submerged_tri.w;
	}

	// volume caclulations done in local space
	if (!Math::is_zero_approx(_submerged_volume)) {
		_submerged_centroid /= (_submerged_volume * 4.0f / 3.0f);
		_submerged_centroid = _collider->get_transform().xform(_submerged_centroid);
	} else {
		_submerged_centroid = _collider->get_transform().origin;
	}
}


#pragma region Triangle Calculations


Vector4 Buoyancy::_tri_contribution(const Vector3 &a, const Vector3 &b, const Vector3 &c, const Vector3 &o) const {
	float V = _sign * (a - o).cross(b - o).dot(c - o) / 6.0f;
	Vector3 C = V * (a + b + c + o) / 4.0f;
	return Vector4(C.x, C.y, C.z, V);
}

Vector3 Buoyancy::_intersect(const Vector3 &v1, const Vector3 &v2, float d1, float d2) const {
	if (Math::is_equal_approx(d1, d2)) {
		return v1;
	}
	float t = d1 / (d1 - d2);
	return v1.lerp(v2, t);
}


// This funection calculates the submerged volume and centroid contribution of a triangle
// given the depths of its vertices. It handles partial submersion by clipping the triangle
// against the liquid plane.
//
// It also adds the submerged vertices to the debug array if enabled so they can be visualized.
//
Vector4 Buoyancy::_partial_intersection(const Vector3 &a, const Vector3 &b, const Vector3 &c, const Vector3 &o,
		float _a, float _b, float _c, bool keep_below) const {
	Vector3 C = Vector3(0, 0, 0);
	float V = 0.0f;

	bool below_a = _a <= 0.0f;
	bool below_b = _b <= 0.0f;
	bool below_c = _c <= 0.0f;
	if (!keep_below) {
		below_a = !below_a;
		below_b = !below_b;
		below_c = !below_c;
	}
	int count = (int)below_a + (int)below_b + (int)below_c;

	// Trivial cases
	if (count == 3) {
		// below
		if (_show_debug) {
			const_cast<PackedVector3Array&>(_submerged_verts).append(a);
			const_cast<PackedVector3Array&>(_submerged_verts).append(b);
			const_cast<PackedVector3Array&>(_submerged_verts).append(c);
		}

		return _tri_contribution(a, b, c, o);
	} else if (count == 0) {
		// above
		return Vector4(0, 0, 0, 0);
	}

	// One vertex below -> single clipped triangle
	if (count == 1) {
		if (below_a) {
			Vector3 p1 = _intersect(a, b, _a, _b);
			Vector3 p2 = _intersect(a, c, _a, _c);

			if (_show_debug) {
				const_cast<PackedVector3Array&>(_submerged_verts).append(a);
				const_cast<PackedVector3Array&>(_submerged_verts).append(p1);
				const_cast<PackedVector3Array&>(_submerged_verts).append(p2);
			}

			return _tri_contribution(a, p1, p2, o);
		} else if (below_b) {
			Vector3 p1 = _intersect(b, c, _b, _c);
			Vector3 p2 = _intersect(b, a, _b, _a);
			if (_show_debug) {
				const_cast<PackedVector3Array&>(_submerged_verts).append(b);
				const_cast<PackedVector3Array&>(_submerged_verts).append(p1);
				const_cast<PackedVector3Array&>(_submerged_verts).append(p2);
			}
			return _tri_contribution(b, p1, p2, o);
		} else {
			Vector3 p1 = _intersect(c, a, _c, _a);
			Vector3 p2 = _intersect(c, b, _c, _b);
			if (_show_debug) {
				const_cast<PackedVector3Array&>(_submerged_verts).append(c);
				const_cast<PackedVector3Array&>(_submerged_verts).append(p1);
				const_cast<PackedVector3Array&>(_submerged_verts).append(p2);
			}
			return _tri_contribution(c, p1, p2, o);
		}
	}

	// Two vertices below -> quad (two triangles)
	else {
		if (!below_a) {
			// b and c below, a above
			Vector3 p1 = _intersect(b, a, _b, _a);
			Vector3 p2 = _intersect(c, a, _c, _a);
			if (_show_debug) {
				const_cast<PackedVector3Array&>(_submerged_verts).append(b);
				const_cast<PackedVector3Array&>(_submerged_verts).append(c);
				const_cast<PackedVector3Array&>(_submerged_verts).append(p2);
				const_cast<PackedVector3Array&>(_submerged_verts).append(b);
				const_cast<PackedVector3Array&>(_submerged_verts).append(p2);
				const_cast<PackedVector3Array&>(_submerged_verts).append(p1);
			}
			Vector4 tri1 = _tri_contribution(b, c, p2, o);
			Vector4 tri2 = _tri_contribution(b, p2, p1, o);
			C += Vector3(tri1.x, tri1.y, tri1.z);
			V += tri1.w;
			C += Vector3(tri2.x, tri2.y, tri2.z);
			V += tri2.w;
		} else if (!below_b) {
			// c and a below, b above
			Vector3 p1 = _intersect(c, b, _c, _b);
			Vector3 p2 = _intersect(a, b, _a, _b);
			if (_show_debug) {
				const_cast<PackedVector3Array&>(_submerged_verts).append(c);
				const_cast<PackedVector3Array&>(_submerged_verts).append(a);
				const_cast<PackedVector3Array&>(_submerged_verts).append(p2);
				const_cast<PackedVector3Array&>(_submerged_verts).append(c);
				const_cast<PackedVector3Array&>(_submerged_verts).append(p2);
				const_cast<PackedVector3Array&>(_submerged_verts).append(p1);
			}
			Vector4 tri1 = _tri_contribution(c, a, p2, o);
			Vector4 tri2 = _tri_contribution(c, p2, p1, o);
			C += Vector3(tri1.x, tri1.y, tri1.z);
			V += tri1.w;
			C += Vector3(tri2.x, tri2.y, tri2.z);
			V += tri2.w;
		} else {
			// a and b below, c above
			Vector3 p1 = _intersect(a, c, _a, _c);
			Vector3 p2 = _intersect(b, c, _b, _c);
			if (_show_debug) {
				const_cast<PackedVector3Array&>(_submerged_verts).append(a);
				const_cast<PackedVector3Array&>(_submerged_verts).append(b);
				const_cast<PackedVector3Array&>(_submerged_verts).append(p2);
				const_cast<PackedVector3Array&>(_submerged_verts).append(a);
				const_cast<PackedVector3Array&>(_submerged_verts).append(p2);
				const_cast<PackedVector3Array&>(_submerged_verts).append(p1);
			}
			Vector4 tri1 = _tri_contribution(a, b, p2, o);
			Vector4 tri2 = _tri_contribution(a, p2, p1, o);
			C += Vector3(tri1.x, tri1.y, tri1.z);
			V += tri1.w;
			C += Vector3(tri2.x, tri2.y, tri2.z);
			V += tri2.w;
		}

		return Vector4(C.x, C.y, C.z, V);
	}
}

#pragma region Forces Application

void Buoyancy::apply_buoyancy_mesh_forces(RigidBody3D *body, float delta) {
	if (!body) return;

	float liquid_density =  _liquid_area ? _liquid_area->get_density() : 1000.0f;
	const Vector3 gravity = body->get_gravity() / body->get_gravity_scale();

	// Calculate buoyant force
	// F_B = rho * V_B * g * (0,1,0)
	float ratio = _submerged_volume / _mesh_volume;
	Vector3 wave_normal = _buoyancy_normal.lerp(Vector3(0, 1, 0), ratio);
	Vector3 buoyant_force = wave_normal * liquid_density * _submerged_volume * -gravity.y;
	Vector3 submerged_position = body->get_global_transform().xform(_submerged_centroid);

	if (_submerged_volume > 0.0) {
		body->apply_force(buoyant_force, submerged_position - body->get_global_position());

		// Ocean currents
		if (_liquid_area) {
			Vector3 current_force = _liquid_area->get_current_speed() * body->get_mass();
			if (current_force.length() > 0.0) {
				body->apply_central_force(current_force);
			}
		}

		// Drag is applied axis aligned with the rigid body
		Basis basis = body->get_global_transform().basis.orthonormalized();
		Vector3 one = Vector3(1,1,1);

		// Underwater drag. We must get the current state directly from the Physics server.
		Vector3 linear_drag = one - _submerged_linear_drag * _linear_drag_scale * delta * ratio;
		Vector3 linear_velocity = PhysicsServer3D::get_singleton()->body_get_state(body->get_rid(), PhysicsServer3D::BODY_STATE_LINEAR_VELOCITY);
		// Vector3 linear_velocity = body->get_linear_velocity();
		
		Vector3 local_vel = basis.xform_inv(linear_velocity);
		Vector3 global_vel = basis.xform(local_vel * linear_drag);

		//PhysicsServer3D::get_singleton()->body_set_state(body->get_rid(), PhysicsServer3D::BODY_STATE_LINEAR_VELOCITY, global_vel);
		body->set_linear_velocity(global_vel);

		Vector3 angular_drag = one - _submerged_angular_drag * _angular_drag_scale * delta * ratio;
		Vector3 angular_velocity = PhysicsServer3D::get_singleton()->body_get_state(body->get_rid(), PhysicsServer3D::BODY_STATE_ANGULAR_VELOCITY);
		// Vector3 angular_velocity = body->get_angular_velocity();
		Vector3 local_ang = basis.xform_inv(angular_velocity);
		Vector3 global_ang = basis.xform(local_ang * angular_drag);

		//PhysicsServer3D::get_singleton()->body_set_state(body->get_rid(), PhysicsServer3D::BODY_STATE_ANGULAR_VELOCITY, global_ang);
		body->set_angular_velocity(global_ang);
	}
}

void Buoyancy::apply_buoyancy_probe_forces(RigidBody3D *body, float delta) {
	if (body == nullptr) return;
	if (_liquid_area == nullptr) return;
	if (_buoyancy_probes.size() == 0) return;

	float liquid_density = _liquid_area->get_density();
	const Vector3 gravity = body->get_gravity() / body->get_gravity_scale();
	bool submerged = false;

	// Reset submerged probe count
	_submerged_probe_count = 0;

	// Get body transform
	Transform3D body_transform = body->get_global_transform();

	// Resize cache for wave transforms
	_probe_wave_transforms.resize(_buoyancy_probes.size());
	_debug_mesh_dirty = true;

	for (int idx = 0; idx < _buoyancy_probes.size(); ++idx) {
		// Get probe position in global space
		Vector3 probe = _buoyancy_probes[idx];
		probe = body_transform.xform(probe);

		// Get liquid position (flat tidal coordinate)
		Vector3 liquid_pos = probe;
		liquid_pos.y = _liquid_area->get_global_transform().origin.y;

		// Get wave transform at this position
		Transform3D wave_xform = Transform3D(Basis(), liquid_pos);
		if (!_ignore_waves) {
			wave_xform = _liquid_area->get_liquid_transform(liquid_pos);
		}
		Vector3 wave_pos = wave_xform.origin;

		// Cache the wave transform, in body local space
		_probe_wave_transforms[idx] = body_transform.affine_inverse() * wave_xform;

		// Calculate depths
		float wave_depth = probe.y - wave_pos.y;
		float liquid_depth = probe.y - liquid_pos.y;

		// Each probe affects 1/N of the mass
		float probe_mass = (body->get_mass() / _buoyancy_probes.size()) * _probe_buoyancy;

		Vector3 liquid_force = Vector3();
		Vector3 wave_force = Vector3();

		// Calculate forces
		liquid_force = gravity * probe_mass * liquid_depth;
		wave_force = wave_xform.basis.get_column(1) * gravity * probe_mass * wave_depth;

		// Apply forces if submerged
		if (wave_depth < 0.0f) {
			submerged = true;
			body->apply_force(wave_force, probe - body_transform.origin);
		}

		if (liquid_depth < 0.0f) {
			submerged = true;
			body->apply_force(liquid_force, probe - body_transform.origin);
		}

		if (submerged) {
			// Count this probe as submerged
			_submerged_probe_count++;
			// Apply current forces
			Vector3 current_force = _liquid_area->get_current_speed() * probe_mass;
			if (current_force.length() > 0.0f) {
				body->apply_force(current_force, probe - body_transform.origin);
			}

			// Apply submerged drag per probe
			Basis basis = body_transform.basis.orthonormalized();
			Vector3 one = Vector3(1, 1, 1);
			float probe_ratio = 1.0f / _buoyancy_probes.size();

			Vector3 linear_drag = one - _submerged_linear_drag * _linear_drag_scale * delta * probe_ratio;
			Vector3 linear_velocity = body->get_linear_velocity();
			Vector3 local_vel = basis.xform_inv(linear_velocity);
			Vector3 global_vel = basis.xform(local_vel * linear_drag);
			body->set_linear_velocity(global_vel);

			Vector3 angular_drag = one - _submerged_angular_drag * _angular_drag_scale * delta * probe_ratio;
			Vector3 angular_velocity = body->get_angular_velocity();
			Vector3 local_ang = basis.xform_inv(angular_velocity);
			Vector3 global_ang = basis.xform(local_ang * angular_drag);
			body->set_angular_velocity(global_ang);
		}
	}
}


#pragma region Debugging

void Buoyancy::_create_debug_mesh() {
	Node3D* parent = Object::cast_to<Node3D>(get_parent());

	if (parent == nullptr) return;
	if (is_inside_tree() == false) return;
	

	if (!_debug_mesh_instance) {
		_debug_mesh_instance = memnew(MeshInstance3D);
		_debug_mesh_instance->set_name(get_name() + String("_DebugMesh"));
		
		// add the child to the parent so it gets the same transform
		parent->add_child(_debug_mesh_instance, true, INTERNAL_MODE_BACK);

		// attach mesh
		if (!_debug_mesh.is_valid()) {
			_debug_mesh.instantiate();
		}
		_debug_mesh_instance->set_mesh(_debug_mesh);
		_debug_mesh_dirty = true;
	}
}

// This call has trash performace but it recreates these arrays each frame
void Buoyancy::_update_debug_mesh() {

	if( _debug_mesh_instance == nullptr) return;
	if( _debug_mesh.is_valid() == false) return;

	if (_debug_mesh_instance && _debug_mesh.is_valid()) {
		PackedVector3Array vertices;
		PackedInt32Array indices;

		// clear previous surfaces
		_debug_mesh->clear_surfaces();

		// Use the submerged verts if available, the original mesh otherwise
		Array arrays;
		arrays.resize(Mesh::ARRAY_MAX);
		Color color = _debug_color;
		Transform3D xform = Transform3D();

		if (_buoyancy_mode == BUOYANCY_COLLIDER) {
			if (!_collider || !_buoyancy_mesh.is_valid()) {
				return;
			}

			xform = _collider->get_global_transform();

			PackedVector3Array verts = _submerged_verts;
			if (verts.size() == 0) {
				verts = _vertex;
				color = Color(0.0, 0.2, 1.0, 0.5);
			}

			int face_count = verts.size() / 3;

			for (int idx = 0; idx < face_count; ++idx) {
				Vector3 a = verts[idx * 3 + 0];
				Vector3 b = verts[idx * 3 + 1];
				Vector3 c = verts[idx * 3 + 2];
				vertices.append(a);
				vertices.append(b);
				vertices.append(c);

				indices.append(idx * 3 + 0);
				indices.append(idx * 3 + 1);

				indices.append(idx * 3 + 1);
				indices.append(idx * 3 + 2);

				indices.append(idx * 3 + 2);
				indices.append(idx * 3 + 0);
			}

			// no submerged verts, skip
			if (face_count == 0)
				return;

			arrays[Mesh::ARRAY_VERTEX] = vertices;
			arrays[Mesh::ARRAY_INDEX] = indices;
		} else {
			// get the parent rigid body
			RigidBody3D *body = Object::cast_to<RigidBody3D>(get_parent());
			if (!body) return;

			xform = body->get_global_transform();

			// no probes, skip
			int probe_count = _buoyancy_probes.size();
			if (probe_count == 0)
				return;

			// Show all probes
			for (int idx = 0; idx < probe_count; ++idx) {
				Vector3 probe = _buoyancy_probes[idx];
				
				// Draw probe cross-hairs
				vertices.append(probe + Vector3(-0.1f, 0, 0));
				vertices.append(probe + Vector3(0.1f, 0, 0));

				vertices.append(probe + Vector3(0, -0.1f, 0));
				vertices.append(probe + Vector3(0, 0.1f, 0));

				vertices.append(probe + Vector3(0, 0, -0.1f));
				vertices.append(probe + Vector3(0, 0, 0.1f));

				indices.append(idx * 6 + 0);
				indices.append(idx * 6 + 1);

				indices.append(idx * 6 + 2);
				indices.append(idx * 6 + 3);

				indices.append(idx * 6 + 4);
				indices.append(idx * 6 + 5);
			}

			// Show wave normals
			for(int idx =0; idx < _probe_wave_transforms.size(); ++idx) {
				Transform3D wave_xform = _probe_wave_transforms[idx];

				// Draw wave normal at probe
				Vector3 origin = wave_xform.origin;
				Vector3 normal_end = origin + wave_xform.basis.get_column(1) * 0.5f;

				vertices.append(origin);
				vertices.append(normal_end);

				indices.append(probe_count * 6 + idx * 2 + 0);
				indices.append(probe_count * 6 + idx * 2 + 1);
			}

			arrays[Mesh::ARRAY_VERTEX] = vertices;
			arrays[Mesh::ARRAY_INDEX] = indices;
		}

		int surf_lines = _debug_mesh->get_surface_count();
		_debug_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_LINES, arrays);

		Ref<StandardMaterial3D> mat;
		mat.instantiate();
		mat->set_shading_mode(StandardMaterial3D::SHADING_MODE_UNSHADED);
		mat->set_depth_draw_mode(StandardMaterial3D::DEPTH_DRAW_DISABLED);
		mat->set_transparency(StandardMaterial3D::TRANSPARENCY_ALPHA);
		mat->set_albedo(color);

		// disable depth test in game so we can see the mesh
		if (!Engine::get_singleton()->is_editor_hint()) {
			mat->set_flag(StandardMaterial3D::FLAG_DISABLE_DEPTH_TEST, true);
		}
		
		_debug_mesh->surface_set_material(surf_lines, mat);

		// move debug mesh
		_debug_mesh_instance->set_global_transform(xform);
	}
}

void Buoyancy::_destroy_debug_mesh() {
	if (_debug_mesh_instance) {
		_debug_mesh_instance->queue_free();
		_debug_mesh_instance = nullptr;
	}

	if (_debug_mesh.is_valid()) {
		_debug_mesh.unref();
	}

	_submerged_verts.clear();
}


#pragma region Bindings

void Buoyancy::_bind_methods() {
	// Property bindings
	ClassDB::bind_method(D_METHOD("set_liquid_area", "liquid_area"), &Buoyancy::set_liquid_area);
	ClassDB::bind_method(D_METHOD("get_liquid_area"), &Buoyancy::get_liquid_area);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "liquid_area", PROPERTY_HINT_NODE_TYPE, "LiquidArea"), "set_liquid_area", "get_liquid_area");

	// Buoyancy mode
	ClassDB::bind_method(D_METHOD("set_buoyancy_mode", "buoyancy_mode"), &Buoyancy::set_buoyancy_mode);
	ClassDB::bind_method(D_METHOD("get_buoyancy_mode"), &Buoyancy::get_buoyancy_mode);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "buoyancy_mode", PROPERTY_HINT_ENUM, "BUOYANCY_COLLIDER:0,BUOYANCY_PROBES:1"), "set_buoyancy_mode", "get_buoyancy_mode");

	BIND_ENUM_CONSTANT(BUOYANCY_COLLIDER);
	BIND_ENUM_CONSTANT(BUOYANCY_PROBES);

	ClassDB::bind_method(D_METHOD("set_collider", "collider"), &Buoyancy::set_collider);
	ClassDB::bind_method(D_METHOD("get_collider"), &Buoyancy::get_collider);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "collider", PROPERTY_HINT_NODE_TYPE, "CollisionShape3D"), "set_collider", "get_collider");

	// Buoyancy probes
	ClassDB::bind_method(D_METHOD("set_buoyancy_probes", "buoyancy_probes"), &Buoyancy::set_buoyancy_probes);
	ClassDB::bind_method(D_METHOD("get_buoyancy_probes"), &Buoyancy::get_buoyancy_probes);
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_VECTOR3_ARRAY, "buoyancy_probes"), "set_buoyancy_probes", "get_buoyancy_probes");


	// ---
	ADD_GROUP("Physics", "");

	ClassDB::bind_method(D_METHOD("set_apply_forces", "ignore_waves"), &Buoyancy::set_apply_forces);
	ClassDB::bind_method(D_METHOD("get_apply_forces"), &Buoyancy::get_apply_forces);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "apply_forces"), "set_apply_forces", "get_apply_forces");

	ClassDB::bind_method(D_METHOD("set_probe_buoyancy", "probe_buoyancy"), &Buoyancy::set_probe_buoyancy);
	ClassDB::bind_method(D_METHOD("get_probe_buoyancy"), &Buoyancy::get_probe_buoyancy);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "probe_buoyancy"), "set_probe_buoyancy", "get_probe_buoyancy");

	ClassDB::bind_method(D_METHOD("set_submerged_linear_drag", "submerged_linear_drag"), &Buoyancy::set_submerged_linear_drag);
	ClassDB::bind_method(D_METHOD("get_submerged_linear_drag"), &Buoyancy::get_submerged_linear_drag);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "submerged_linear_drag"), "set_submerged_linear_drag", "get_submerged_linear_drag");

	ClassDB::bind_method(D_METHOD("set_submerged_angular_drag", "submerged_angular_drag"), &Buoyancy::set_submerged_angular_drag);
	ClassDB::bind_method(D_METHOD("get_submerged_angular_drag"), &Buoyancy::get_submerged_angular_drag);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "submerged_angular_drag"), "set_submerged_angular_drag", "get_submerged_angular_drag");

	ClassDB::bind_method(D_METHOD("set_linear_drag_scale", "linear_drag_scale"), &Buoyancy::set_linear_drag_scale);
	ClassDB::bind_method(D_METHOD("get_linear_drag_scale"), &Buoyancy::get_linear_drag_scale);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "linear_drag_scale"), "set_linear_drag_scale", "get_linear_drag_scale");

	ClassDB::bind_method(D_METHOD("set_angular_drag_scale", "angular_drag_scale"), &Buoyancy::set_angular_drag_scale);
	ClassDB::bind_method(D_METHOD("get_angular_drag_scale"), &Buoyancy::get_angular_drag_scale);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "angular_drag_scale"), "set_angular_drag_scale", "get_angular_drag_scale");

	ClassDB::bind_method(D_METHOD("set_ignore_waves", "ignore_waves"), &Buoyancy::set_ignore_waves);
	ClassDB::bind_method(D_METHOD("get_ignore_waves"), &Buoyancy::get_ignore_waves);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "ignore_waves"), "set_ignore_waves", "get_ignore_waves");

	// Read only
	ClassDB::bind_method(D_METHOD("get_submerged_volume"), &Buoyancy::get_submerged_volume);
	ClassDB::bind_method(D_METHOD("get_submerged_centroid"), &Buoyancy::get_submerged_centroid);
	ClassDB::bind_method(D_METHOD("get_buoyancy_normal"), &Buoyancy::get_buoyancy_normal);
	ClassDB::bind_method(D_METHOD("get_submerged_ratio"), &Buoyancy::get_submerged_ratio);

	ClassDB::bind_method(D_METHOD("get_volume"), &Buoyancy::get_volume);
	ClassDB::bind_method(D_METHOD("get_centroid"), &Buoyancy::get_centroid);

	// Info properties (read-only)
	ClassDB::bind_method(D_METHOD("get_mass"), &Buoyancy::get_mass);
	ClassDB::bind_method(D_METHOD("get_center_of_mass"), &Buoyancy::get_center_of_mass);
	ClassDB::bind_method(D_METHOD("get_inertia"), &Buoyancy::get_inertia);

	ClassDB::bind_method(D_METHOD("_get_buoyancy_time"), &Buoyancy::_get_buoyancy_time);

	// Function calls
	ClassDB::bind_method(D_METHOD("apply_buoyancy_mesh_forces"), &Buoyancy::apply_buoyancy_mesh_forces);
	ClassDB::bind_method(D_METHOD("apply_buoyancy_probe_forces"), &Buoyancy::apply_buoyancy_probe_forces);


	// ---
	ADD_GROUP("Mass Properties", "");

	ClassDB::bind_method(D_METHOD("set_calculate_mass_properties", "calculate_mass_properties"), &Buoyancy::set_calculate_mass_properties);
	ClassDB::bind_method(D_METHOD("get_calculate_mass_properties"), &Buoyancy::get_calculate_mass_properties);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "calculate_mass_properties"), "set_calculate_mass_properties", "get_calculate_mass_properties");

	ClassDB::bind_method(D_METHOD("set_density", "density"), &Buoyancy::set_density);
	ClassDB::bind_method(D_METHOD("get_density"), &Buoyancy::get_density);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "density", PROPERTY_HINT_RANGE, "1,10000,1,suffix:kg*m^3"), "set_density", "get_density");

	ClassDB::bind_method(D_METHOD("set_com_offset", "com_offset"), &Buoyancy::set_com_offset);
	ClassDB::bind_method(D_METHOD("get_com_offset"), &Buoyancy::get_com_offset);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "com_offset"), "set_com_offset", "get_com_offset");


	// ---
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "volume", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_READ_ONLY | PROPERTY_USAGE_NO_INSTANCE_STATE | PROPERTY_USAGE_EDITOR), "", "get_volume");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "centroid", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_READ_ONLY | PROPERTY_USAGE_NO_INSTANCE_STATE | PROPERTY_USAGE_EDITOR), "", "get_centroid");

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "body_mass", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_READ_ONLY | PROPERTY_USAGE_NO_INSTANCE_STATE | PROPERTY_USAGE_EDITOR), "", "get_mass");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "body_center_of_mass", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_READ_ONLY | PROPERTY_USAGE_NO_INSTANCE_STATE | PROPERTY_USAGE_EDITOR), "", "get_center_of_mass");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "body_inertia", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_READ_ONLY | PROPERTY_USAGE_NO_INSTANCE_STATE | PROPERTY_USAGE_EDITOR), "", "get_inertia");


	// Debug
	ADD_GROUP("Debug", "");
	ClassDB::bind_method(D_METHOD("set_show_debug", "show_debug"), &Buoyancy::set_show_debug);
	ClassDB::bind_method(D_METHOD("get_show_debug"), &Buoyancy::get_show_debug);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "show_debug"), "set_show_debug", "get_show_debug");

	ClassDB::bind_method(D_METHOD("set_debug_color", "debug_color"), &Buoyancy::set_debug_color);
	ClassDB::bind_method(D_METHOD("get_debug_color"), &Buoyancy::get_debug_color);
	ADD_PROPERTY(PropertyInfo(Variant::COLOR, "debug_color"), "set_debug_color", "get_debug_color");

	// Internal methods
	// ClassDB::bind_method(D_METHOD("_property_changed", "prop"), &Buoyancy::_property_changed);

	// Signals
	ADD_SIGNAL(MethodInfo("submerged_changed"));

	// virtuals
	// GDVIRTUAL_BIND(get_configuration_warnings)
}