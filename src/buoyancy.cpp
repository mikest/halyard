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

Buoyancy::Buoyancy() {
}

Buoyancy::~Buoyancy() {
}

void Buoyancy::_bind_methods() {
	// Property bindings
	ClassDB::bind_method(D_METHOD("set_liquid_area", "liquid_area"), &Buoyancy::set_liquid_area);
	ClassDB::bind_method(D_METHOD("get_liquid_area"), &Buoyancy::get_liquid_area);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "liquid_area", PROPERTY_HINT_NODE_TYPE, "LiquidArea"), "set_liquid_area", "get_liquid_area");

	ClassDB::bind_method(D_METHOD("set_collider", "collider"), &Buoyancy::set_collider);
	ClassDB::bind_method(D_METHOD("get_collider"), &Buoyancy::get_collider);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "collider", PROPERTY_HINT_NODE_TYPE, "CollisionShape3D"), "set_collider", "get_collider");

	ClassDB::bind_method(D_METHOD("set_density", "density"), &Buoyancy::set_density);
	ClassDB::bind_method(D_METHOD("get_density"), &Buoyancy::get_density);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "density", PROPERTY_HINT_RANGE, "1,10000,1,suffix:kg*m^3"), "set_density", "get_density");

	ClassDB::bind_method(D_METHOD("set_mass_scale", "mass_scale"), &Buoyancy::set_mass_scale);
	ClassDB::bind_method(D_METHOD("get_mass_scale"), &Buoyancy::get_mass_scale);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "mass_scale", PROPERTY_HINT_RANGE, "0.001,10.0,0.001"), "set_mass_scale", "get_mass_scale");

	ClassDB::bind_method(D_METHOD("set_com_offset", "com_offset"), &Buoyancy::set_com_offset);
	ClassDB::bind_method(D_METHOD("get_com_offset"), &Buoyancy::get_com_offset);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "com_offset"), "set_com_offset", "get_com_offset");

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

	ClassDB::bind_method(D_METHOD("set_calculate_center_of_mass", "calculate_center_of_mass"), &Buoyancy::set_calculate_center_of_mass);
	ClassDB::bind_method(D_METHOD("get_calculate_center_of_mass"), &Buoyancy::get_calculate_center_of_mass);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "calculate_center_of_mass"), "set_calculate_center_of_mass", "get_calculate_center_of_mass");

	ClassDB::bind_method(D_METHOD("set_ignore_waves", "ignore_waves"), &Buoyancy::set_ignore_waves);
	ClassDB::bind_method(D_METHOD("get_ignore_waves"), &Buoyancy::get_ignore_waves);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "ignore_waves"), "set_ignore_waves", "get_ignore_waves");

	ClassDB::bind_method(D_METHOD("set_enabled", "enabled"), &Buoyancy::set_enabled);
	ClassDB::bind_method(D_METHOD("get_enabled"), &Buoyancy::get_enabled);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "enabled"), "set_enabled", "get_enabled");

	// Info properties (read-only)
	ClassDB::bind_method(D_METHOD("get_mass"), &Buoyancy::get_mass);
	ClassDB::bind_method(D_METHOD("get_volume"), &Buoyancy::get_volume);
	ClassDB::bind_method(D_METHOD("get_center_of_mass"), &Buoyancy::get_center_of_mass);
	ClassDB::bind_method(D_METHOD("get_inertia"), &Buoyancy::get_inertia);

	ADD_GROUP("Info", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "mass", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_READ_ONLY | PROPERTY_USAGE_NO_INSTANCE_STATE | PROPERTY_USAGE_EDITOR), "", "get_mass");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "volume", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_READ_ONLY | PROPERTY_USAGE_NO_INSTANCE_STATE | PROPERTY_USAGE_EDITOR), "", "get_volume");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "center_of_mass", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_READ_ONLY | PROPERTY_USAGE_NO_INSTANCE_STATE | PROPERTY_USAGE_EDITOR), "", "get_center_of_mass");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "inertia", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_READ_ONLY | PROPERTY_USAGE_NO_INSTANCE_STATE | PROPERTY_USAGE_EDITOR), "", "get_inertia");

	// Internal methods
	ClassDB::bind_method(D_METHOD("_property_changed", "prop"), &Buoyancy::_property_changed);
}

void Buoyancy::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_READY: {
			if (Engine::get_singleton()->is_editor_hint()) {
				// Connect to inspector property edited signal if in editor
				// Note: This might need adjustment based on Godot's API
			}
			_update_statics();
			set_physics_process(true);
			
		} break;
		case NOTIFICATION_PHYSICS_PROCESS: {
			if (!Engine::get_singleton()->is_editor_hint()) {
				RigidBody3D *body = Object::cast_to<RigidBody3D>(get_parent());
				if (body && !body->is_freeze_enabled() && _enabled) {

					uint64_t time = Time::get_singleton()->get_ticks_usec();

					float delta = get_physics_process_delta_time();
					_apply_buoyancy_forces(body, delta);

					uint64_t elapsed = Time::get_singleton()->get_ticks_usec() - time;
					// UtilityFunctions::print("Buoyancy physics process time (usec): ", elapsed);
				}
			}
		} break;
	}
}

// Property getters/setters
void Buoyancy::set_liquid_area(LiquidArea *p_area) {
	_liquid_area = p_area;
}

LiquidArea* Buoyancy::get_liquid_area() const {
	return _liquid_area;
}

void Buoyancy::set_collider(CollisionShape3D *p_collider) {
	_collider = p_collider;
	_update_statics();
}

CollisionShape3D* Buoyancy::get_collider() const {
	return _collider;
}

void Buoyancy::set_density(float p_density) {
	_density = p_density;
	_update_statics();
}

float Buoyancy::get_density() const {
	return _density;
}

void Buoyancy::set_mass_scale(float p_mass_scale) {
	_mass_scale = p_mass_scale;
	_update_statics();
}

float Buoyancy::get_mass_scale() const {
	return _mass_scale;
}

void Buoyancy::set_com_offset(float p_com_offset) {
	_com_offset = p_com_offset;
}

float Buoyancy::get_com_offset() const {
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

void Buoyancy::set_calculate_center_of_mass(bool p_calculate) {
	_calculate_center_of_mass = p_calculate;
	_update_statics();
}

bool Buoyancy::get_calculate_center_of_mass() const {
	return _calculate_center_of_mass;
}

void Buoyancy::set_ignore_waves(bool p_ignore) {
	_ignore_waves = p_ignore;
}

bool Buoyancy::get_ignore_waves() const {
	return _ignore_waves;
}

void Buoyancy::set_enabled(bool p_enabled) {
	_enabled = p_enabled;
}

bool Buoyancy::get_enabled() const {
	return _enabled;
}

// Info getters
float Buoyancy::get_mass() const {
	RigidBody3D *body = Object::cast_to<RigidBody3D>(get_parent());
	return body ? body->get_mass() : -1.0f;
}

float Buoyancy::get_volume() const {
	return _mesh_volume;
}

Vector3 Buoyancy::get_center_of_mass() const {
	RigidBody3D *body = Object::cast_to<RigidBody3D>(get_parent());
	return body ? body->get_center_of_mass() : Vector3(0, 0, 0);
}

Vector3 Buoyancy::get_inertia() const {
	RigidBody3D *body = Object::cast_to<RigidBody3D>(get_parent());
	return body ? body->get_inertia() : Vector3(0, 0, 0);
}

void Buoyancy::_property_changed(const String &prop) {
	static const String UPDATE_PROPS[] = {"collider", "density", "mass_scale", "com_offset", "calculate_center_of_mass"};
	for (const String &update_prop : UPDATE_PROPS) {
		if (prop == update_prop) {
			_update_statics();
			break;
		}
	}
}

void Buoyancy::_on_config_changed() {
	// _show_debug = Engine::get_singleton()->is_editor_hint() || (Config.debug_visible && Config.debug_buoyancy);
	// Need to implement Config access
}

void Buoyancy::_update_statics() {
	if (!is_node_ready()) return;

	ERR_FAIL_NULL_MSG(_collider, "Missing collider mesh for Buoyancy");

	Ref<Shape3D> shape = _collider->get_shape();
	if (shape.is_valid() && Object::cast_to<ConvexPolygonShape3D>(*shape)) {
		_debug_mesh = Object::cast_to<ConvexPolygonShape3D>(*shape)->get_debug_mesh();
	}

	if (_debug_mesh.is_valid() && _debug_mesh->get_surface_count()) {
		_vertex = _debug_mesh->get_faces();

		// Calculate volume and centroid
		_mesh_volume = 0.0f;
		_mesh_centroid = Vector3(0, 0, 0);
		Vector3 o = Vector3(0, 0, 0);

		int face_count = _vertex.size() / 3;
		for (int idx = 0; idx < face_count; ++idx) {
			Vector3 a = _vertex[idx * 3 + 0];
			Vector3 b = _vertex[idx * 3 + 1];
			Vector3 c = _vertex[idx * 3 + 2];

			Vector4 tri = _tri_contribution(a, b, c, o);
			_mesh_volume += tri.w;
			_mesh_centroid += Vector3(tri.x, tri.y, tri.z);
		}

		if (!Math::is_zero_approx(_mesh_volume)) {
			_mesh_centroid /= (_mesh_volume * 4.0f / 3.0f);
			_mesh_centroid = _collider->get_transform().xform(_mesh_centroid);
		} else {
			_mesh_centroid = _collider->get_global_transform().origin;
		}

		// Auto detect negative mass and recalc with sign flipped
		if (_mesh_volume < 0.0f) {
			_sign = 1.0f;
			_mesh_volume = -_mesh_volume;
			_mesh_centroid = Vector3(0, 0, 0);
			for (int idx = 0; idx < face_count; ++idx) {
				Vector3 a = _vertex[idx * 3 + 0];
				Vector3 b = _vertex[idx * 3 + 1];
				Vector3 c = _vertex[idx * 3 + 2];

				Vector4 tri = _tri_contribution(a, b, c, o);
				_mesh_volume += tri.w;
				_mesh_centroid += Vector3(tri.x, tri.y, tri.z);
			}
			if (!Math::is_zero_approx(_mesh_volume)) {
				_mesh_centroid /= (_mesh_volume * 4.0f / 3.0f);
				_mesh_centroid = _collider->get_transform().xform(_mesh_centroid);
			}
		}

		// Update mass from collider
		RigidBody3D *body = Object::cast_to<RigidBody3D>(get_parent());
		if (body && _calculate_center_of_mass) {
			float mass = _mesh_volume * _density * _mass_scale;
			body->set_mass(mass);

			body->set_center_of_mass_mode(RigidBody3D::CENTER_OF_MASS_MODE_CUSTOM);
			body->set_center_of_mass(_mesh_centroid + Vector3(0, _com_offset, 0));

			// Calculate inertia tensor
			float ix = 0.0f, iy = 0.0f, iz = 0.0f;
			// Simplified inertia calculation - this would need proper implementation
			body->set_inertia(Vector3(ix, iy, iz));
		}
	}
}

void Buoyancy::_update_dynamics() {
	// Reset the normal
	_buoyancy_normal = Vector3(0, 0, 0);

	// Get the transformed mesh points and their depths
	_depth_map.clear();
	_depths.resize(_vertex.size());
	for (int idx = 0; idx < _vertex.size(); ++idx) {
		Vector3 key = _vertex[idx];
		Variant val = _depth_map.get(key, Variant());
		Vector3 global_pos = _collider->get_global_transform().xform(_vertex[idx]);
		if (val.get_type() == Variant::NIL) {
			// Get wave transform from ocean
			float liquid_y = _liquid_area ? _liquid_area->get_global_transform().origin.y : 0.0f;
			Transform3D wave_xform = Transform3D(Basis(), Vector3(global_pos.x, liquid_y, global_pos.z));
			if (!_ignore_waves && _liquid_area) {
				Transform3D result = _liquid_area->get_liquid_transform(global_pos);
				wave_xform = result;
			}
			_depth_map[key] = wave_xform;
			val = wave_xform;
		}
		Transform3D xform = val;
		_depths[idx] = global_pos.y - xform.origin.y;
		_buoyancy_normal += xform.basis.get_column(1) / (float)_vertex.size();
	}

	_buoyancy_normal = _buoyancy_normal.normalized();

	// Calculate the locations for each face
	int face_count = _vertex.size() / 3;
	Transform3D T = _collider->get_global_transform();
	_T = T;
	Vector3 o = Vector3(0, 0, 0);
	_submerged_centroid = Vector3(0, 0, 0);
	_submerged_volume = 0.0f;
	_gravity_centroid = Vector3(0, 0, 0);
	_gravity_volume = 0.0f;

	for (int idx = 0; idx < face_count; ++idx) {
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

		// Calculate gravity contribution
		Vector4 gravity_tri = _partial_intersection(a, b, c, o, _a, _b, _c, false);
		_gravity_centroid += Vector3(gravity_tri.x, gravity_tri.y, gravity_tri.z);
		_gravity_volume += gravity_tri.w;
	}

	if (!Math::is_zero_approx(_submerged_volume)) {
		_submerged_centroid /= (_submerged_volume * 4.0f / 3.0f);
		_submerged_centroid = _collider->get_transform().xform(_submerged_centroid);
	} else {
		_submerged_centroid = _collider->get_global_transform().origin;
	}

	if (!Math::is_zero_approx(_gravity_volume)) {
		_gravity_centroid /= (_gravity_volume * 4.0f / 3.0f);
		_gravity_centroid = _collider->get_transform().xform(_gravity_centroid);
	} else {
		_gravity_centroid = _collider->get_global_transform().origin;
	}
}

Vector4 Buoyancy::_tri_contribution(const Vector3 &a, const Vector3 &b, const Vector3 &c, const Vector3 &o) const {
	float V = _sign * (a - o).cross(b - o).dot(c - o) / 6.0f;
	Vector3 C = V * (a + b + c + o) / 4.0f;

	// Debug drawing would go here if needed

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
		Vector4 tri = _tri_contribution(a, b, c, o);
		C = Vector3(tri.x, tri.y, tri.z);
		V = tri.w;
		return Vector4(C.x, C.y, C.z, V);
	} else if (count == 0) {
		return Vector4(0, 0, 0, 0);
	}

	// One vertex below -> single clipped triangle
	if (count == 1) {
		if (below_a) {
			Vector3 p1 = _intersect(a, b, _a, _b);
			Vector3 p2 = _intersect(a, c, _a, _c);
			Vector4 tri = _tri_contribution(a, p1, p2, o);
			C += Vector3(tri.x, tri.y, tri.z);
			V += tri.w;
		} else if (below_b) {
			Vector3 p1 = _intersect(b, c, _b, _c);
			Vector3 p2 = _intersect(b, a, _b, _a);
			Vector4 tri = _tri_contribution(b, p1, p2, o);
			C += Vector3(tri.x, tri.y, tri.z);
			V += tri.w;
		} else {
			Vector3 p1 = _intersect(c, a, _c, _a);
			Vector3 p2 = _intersect(c, b, _c, _b);
			Vector4 tri = _tri_contribution(c, p1, p2, o);
			C += Vector3(tri.x, tri.y, tri.z);
			V += tri.w;
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
	}

	return Vector4(C.x, C.y, C.z, V);
}

void Buoyancy::_apply_buoyancy_forces(RigidBody3D *body, float delta) {
	if (!body) return;

	_update_dynamics();

	const float WATER_DENSITY = 1000.0f;
	const float GRAVITY = 9.8f; // Assuming default gravity

	// Calculate buoyant force
	// F_B = rho * V_B * g * (0,1,0)
	Vector3 wave_normal = _buoyancy_normal.lerp(Vector3(0, 1, 0), _submerged_volume / _mesh_volume);
	Vector3 buoyant_force = wave_normal * WATER_DENSITY * _submerged_volume * GRAVITY;
	Vector3 submerged_position = body->get_global_transform().xform(_submerged_centroid);

	if (_submerged_volume > 0.0f) {
		body->apply_force(buoyant_force, submerged_position - body->get_global_position());

		// Underwater drag
		float ratio = _submerged_volume / _mesh_volume;
		Vector3 linear_drag = Vector3(
			Math::exp(-_submerged_linear_drag * _linear_drag_scale.x * delta),
			Math::exp(-_submerged_linear_drag * _linear_drag_scale.y * delta),
			Math::exp(-_submerged_linear_drag * _linear_drag_scale.z * delta)
		);
		linear_drag = Vector3(1, 1, 1).lerp(linear_drag, ratio);

		// Apply per-axis drag in the body's local axes
		Basis basis = body->get_global_transform().basis.orthonormalized();
		Vector3 local_vel = basis.xform_inv(body->get_linear_velocity());
		local_vel *= linear_drag;
		body->set_linear_velocity(basis.xform(local_vel));

		Vector3 angular_drag = Vector3(
			Math::exp(-_submerged_angular_drag * _angular_drag_scale.x * delta),
			Math::exp(-_submerged_angular_drag * _angular_drag_scale.y * delta),
			Math::exp(-_submerged_angular_drag * _angular_drag_scale.z * delta)
		);
		angular_drag = Vector3(1, 1, 1).lerp(angular_drag, ratio);

		Vector3 local_ang = basis.xform_inv(body->get_angular_velocity());
		local_ang *= angular_drag;
		body->set_angular_velocity(basis.xform(local_ang));

		// Ocean currents
		if (_liquid_area) {
			Vector3 current_force = _liquid_area->get_current_speed() * body->get_mass();
			body->apply_central_force(current_force);
		}
	}

	// Update submerged flag for world bodies
	// Note: WorldBody is not implemented in C++, so this part is omitted
}