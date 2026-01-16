#include "buoyancy.h"
#include "liquid_area.h"

#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/rigid_body3d.hpp>
#include <godot_cpp/classes/shape3d.hpp>
#include <godot_cpp/classes/convex_polygon_shape3d.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/classes/performance.hpp>
#include <godot_cpp/classes/rendering_server.hpp>

Buoyancy::Buoyancy() {
}


Buoyancy::~Buoyancy() {
	if (_buoyancy_mesh.is_valid()) {
		_buoyancy_mesh.unref();
	}
}


void Buoyancy::_bind_methods() {
	// Property bindings
	ClassDB::bind_method(D_METHOD("set_liquid_area", "liquid_area"), &Buoyancy::set_liquid_area);
	ClassDB::bind_method(D_METHOD("get_liquid_area"), &Buoyancy::get_liquid_area);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "liquid_area", PROPERTY_HINT_NODE_TYPE, "LiquidArea"), "set_liquid_area", "get_liquid_area");

	ClassDB::bind_method(D_METHOD("set_collider", "collider"), &Buoyancy::set_collider);
	ClassDB::bind_method(D_METHOD("get_collider"), &Buoyancy::get_collider);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "collider", PROPERTY_HINT_NODE_TYPE, "CollisionShape3D"), "set_collider", "get_collider");


	// ---
	ADD_GROUP("Physics", "");

	ClassDB::bind_method(D_METHOD("set_apply_forces", "ignore_waves"), &Buoyancy::set_apply_forces);
	ClassDB::bind_method(D_METHOD("get_apply_forces"), &Buoyancy::get_apply_forces);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "apply_forces"), "set_apply_forces", "get_apply_forces");

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

	ClassDB::bind_method(D_METHOD("get_volume"), &Buoyancy::get_volume);
	ClassDB::bind_method(D_METHOD("get_centroid"), &Buoyancy::get_centroid);

	// Info properties (read-only)
	ClassDB::bind_method(D_METHOD("get_mass"), &Buoyancy::get_mass);
	ClassDB::bind_method(D_METHOD("get_center_of_mass"), &Buoyancy::get_center_of_mass);
	ClassDB::bind_method(D_METHOD("get_inertia"), &Buoyancy::get_inertia);

	ClassDB::bind_method(D_METHOD("_get_buoyancy_time"), &Buoyancy::_get_buoyancy_time);

	// Function calls
	ClassDB::bind_method(D_METHOD("apply_buoyancy_forces"), &Buoyancy::apply_buoyancy_forces);


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

	// Internal methods
	// ClassDB::bind_method(D_METHOD("_property_changed", "prop"), &Buoyancy::_property_changed);

	// virtuals
	// GDVIRTUAL_BIND(get_configuration_warnings)
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
	
	if (!_collider) {
		what.append("Missing collider.");
	} else {
		Ref<Shape3D> shape = _collider->get_shape();
		if (!shape.is_valid()) {
			what.append("Missing collider shape.");
		}else if (Object::cast_to<ConvexPolygonShape3D>(*shape) == nullptr) {
			what.append("Collider shape must be a ConvexPolygonShape3D.");
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
			set_physics_process(true);
		} break;

		case NOTIFICATION_ENTER_TREE: {
			if (Engine::get_singleton()->is_editor_hint()==false) {
				if (_liquid_area == nullptr) {
					SceneTree *tree = get_tree();
					if(tree){
						Node* root = tree->get_current_scene();
						if (root){
							auto nodes = root->find_children("*", "LiquidArea", true, false);
							if (nodes.size()) {
								_liquid_area = Object::cast_to<LiquidArea>(nodes.front());
							}
						}
					}
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
		} break;

		case NOTIFICATION_PHYSICS_PROCESS: {
			if (!Engine::get_singleton()->is_editor_hint() && is_node_ready()) {
				RigidBody3D *body = Object::cast_to<RigidBody3D>(get_parent());
				if (body && !body->is_freeze_enabled()) {

					uint64_t time = Time::get_singleton()->get_ticks_usec();

					// recalculate buoyancy volumes and centroids
					_update_dynamics();

					// optionally apply them
					if (_apply_forces){
						float delta = get_physics_process_delta_time();
						apply_buoyancy_forces(body, delta);
					}

					uint64_t elapsed = Time::get_singleton()->get_ticks_usec() - time;
					_buoyancy_time = elapsed;
					// UtilityFunctions::print("Buoyancy physics process time (usec): ", elapsed);
				}
			}
		} break;
	}
}

// Property getters/setters
void Buoyancy::set_liquid_area(LiquidArea *p_area) {
	_liquid_area = p_area;
	_update_configuration_warnings();
}

LiquidArea* Buoyancy::get_liquid_area() const {
	return _liquid_area;
}

void Buoyancy::set_collider(CollisionShape3D *p_collider) {
	_collider = p_collider;
	_set_dirty();
	_update_configuration_warnings();
}

CollisionShape3D* Buoyancy::get_collider() const {
	return _collider;
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


void Buoyancy::_update_statics() {
	if(!_collider){
		return;
	}

	Ref<Shape3D> shape = _collider->get_shape();
	if (shape.is_valid()) {
		_buoyancy_mesh = shape->get_debug_mesh();
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
	if (!_collider) {
		return;
	}

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
			return _tri_contribution(a, p1, p2, o);
		} else if (below_b) {
			Vector3 p1 = _intersect(b, c, _b, _c);
			Vector3 p2 = _intersect(b, a, _b, _a);
			return _tri_contribution(b, p1, p2, o);
		} else {
			Vector3 p1 = _intersect(c, a, _c, _a);
			Vector3 p2 = _intersect(c, b, _c, _b);
			return _tri_contribution(c, p1, p2, o);
		}
	}

	// Two vertices below -> quad (two triangles)
	else {
		if (!below_a) {
			// b and c below, a above
			Vector3 p1 = _intersect(b, a, _b, _a);
			Vector3 p2 = _intersect(c, a, _c, _a);
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

void Buoyancy::apply_buoyancy_forces(RigidBody3D *body, float delta) {
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