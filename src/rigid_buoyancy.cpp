/* RigidBuoyancy
 *
 * A class for adding Buoyancy to a RigidBody3D.
 *
 * This objects manages the physics interactions between itself and a single LiquidArea in the scene.
 *
 * Copyright (c) M. Estee
 * MIT License.
 */

#include "rigid_buoyancy.h"
#include "halyard_utils.h"
#include "liquid_area.h"

#include <godot_cpp/classes/box_shape3d.hpp>
#include <godot_cpp/classes/capsule_shape3d.hpp>
#include <godot_cpp/classes/convex_polygon_shape3d.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/performance.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/classes/rigid_body3d.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/shape3d.hpp>
#include <godot_cpp/classes/sphere_shape3d.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

#pragma region Lifecycle

RigidBuoyancy::RigidBuoyancy() :
		NodeDebug(Object::cast_to<Node>(this)) {
	_debug_color = Color(0.0f, 0.8f, 1.0f, 0.2f);
}

RigidBuoyancy::~RigidBuoyancy() {
	_destroy_debug_mesh();
}

#pragma endregion

#pragma region Godot Overrides

void RigidBuoyancy::_update_configuration_warnings() {
	if (Engine::get_singleton()->is_editor_hint()) {
		update_configuration_warnings();
	}
}

PackedStringArray RigidBuoyancy::_get_configuration_warnings() const {
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
			} else {
				ConvexPolygonShape3D *convex_shape = Object::cast_to<ConvexPolygonShape3D>(*shape);
				BoxShape3D *box_shape = Object::cast_to<BoxShape3D>(*shape);
				SphereShape3D *sphere_shape = Object::cast_to<SphereShape3D>(*shape);
				CapsuleShape3D *capsule_shape = Object::cast_to<CapsuleShape3D>(*shape);
				if (!convex_shape && !box_shape && !sphere_shape && !capsule_shape) {
					what.append("Collider shape must be a ConvexPolygonShape3D, BoxShape3D, SphereShape3D, or CapsuleShape3D. Other shapes are not supported.");
				}
			}
		}
	} else {
		if (_probe_buoyancy.get_probes().size() == 0) {
			what.append("No buoyancy probes defined for point-based buoyancy.");
		}
	}

	if (!_buoyancy_material.is_valid()) {
		what.append("No buoyancy material assigned. Default values will be used.");
	}

	// NOTE: There are legitimate reasons to not have a liquid area assigned
	return what;
}

void RigidBuoyancy::_notification(int p_what) {
	// manage debugging
	NodeDebug::_debug_notification(p_what);

	switch (p_what) {
		case NOTIFICATION_READY: {
			_set_dirty();
			set_process_internal(true);
			set_physics_process_internal(true);

			// if no buoyancy material is assigned a default one
			if (Engine::get_singleton()->is_editor_hint() == false) {
				if (!_buoyancy_material.is_valid()) {
					_buoyancy_material.instantiate();
				}
			}
		} break;

		case NOTIFICATION_ENTER_TREE: {
			// if (Engine::get_singleton()->is_editor_hint() == false) {
			if (_liquid_area == nullptr) {
				SceneTree *tree = get_tree();
				_liquid_area = LiquidArea::get_liquid_area(tree);
				_probe_buoyancy.set_liquid_area(_liquid_area);
				_mesh_buoyancy.set_liquid_area(_liquid_area);
			}
			// }
		} break;

		case NOTIFICATION_EXIT_TREE: {
			_liquid_area = nullptr;
			_probe_buoyancy.set_liquid_area(nullptr);
			_mesh_buoyancy.set_liquid_area(nullptr);
		} break;

		case NOTIFICATION_INTERNAL_PROCESS: {
			if (_dirty && is_node_ready()) {
				_update_statics();
			}

			if (Engine::get_singleton()->is_editor_hint() == true) {
				if (_show_debug)
					set_debug_mesh_dirty();
			}
		} break;

		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
			if (is_node_ready()) {
				if (Engine::get_singleton()->is_editor_hint()) {
					// update the probe transforms so we can see the sampled locations in the editor
					if (_buoyancy_mode == BUOYANCY_PROBES) {
						_update_last_probe_transforms();
					} else {
						// recalc dynamics
						_update_dynamics();
					}

				} else {
					// in game
					RigidBody3D *body = Object::cast_to<RigidBody3D>(get_parent());
					if (body && !body->is_freeze_enabled()) {
						uint64_t time = Time::get_singleton()->get_ticks_usec();

						// always update submerged state
						if (_buoyancy_mode == BUOYANCY_PROBES) {
							_update_last_probe_transforms();
						} else {
							// recalc dynamics
							_update_dynamics();
						}

						// optionally apply them
						if (_apply_forces) {
							float delta = get_physics_process_delta_time();
							if (_buoyancy_mode == BUOYANCY_PROBES) {
								apply_buoyancy_probe_forces(body, delta);
							} else {
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
			}
		} break;
	}
}

#pragma endregion

#pragma region Properties
void RigidBuoyancy::set_liquid_area(LiquidArea *p_area) {
	_liquid_area = p_area;
	_probe_buoyancy.set_liquid_area(_liquid_area);
	_mesh_buoyancy.set_liquid_area(_liquid_area);
	_update_configuration_warnings();

	_set_dirty();
	set_debug_mesh_dirty();
}

LiquidArea *RigidBuoyancy::get_liquid_area() const {
	return _liquid_area;
}

void RigidBuoyancy::set_collider(CollisionShape3D *p_collider) {
	if (_collider != p_collider) {
		_collider = p_collider;
		_set_dirty();

		_update_configuration_warnings();
	}

	set_debug_mesh_dirty();
}

CollisionShape3D *RigidBuoyancy::get_collider() const {
	return _collider;
}

void RigidBuoyancy::set_buoyancy_material(const Ref<BuoyancyMaterial> &p_material) {
	_buoyancy_material = p_material;
	_set_dirty();
}

Ref<BuoyancyMaterial> RigidBuoyancy::get_buoyancy_material() const {
	return _buoyancy_material;
}

// Buoyancy mode
void RigidBuoyancy::set_buoyancy_mode(BuoyancyMode p_mode) {
	ERR_FAIL_COND_MSG(p_mode != BUOYANCY_COLLIDER && p_mode != BUOYANCY_PROBES, "Invalid buoyancy mode");
	_buoyancy_mode = p_mode;
	set_debug_mesh_dirty();
	_set_dirty();
	_update_configuration_warnings();
}

BuoyancyMode RigidBuoyancy::get_buoyancy_mode() const {
	return _buoyancy_mode;
}

// Buoyancy probes
void RigidBuoyancy::set_probes(const PackedVector3Array &p_probes) {
	_probe_buoyancy.set_probes(p_probes);

	set_debug_mesh_dirty();
	_set_dirty();
	_update_configuration_warnings();
}

PackedVector3Array RigidBuoyancy::get_probes() const {
	return _probe_buoyancy.get_probes();
}

void RigidBuoyancy::set_apply_forces(bool p_apply_forces) {
	_apply_forces = p_apply_forces;
	_update_configuration_warnings();
}

float RigidBuoyancy::get_apply_forces() const {
	return _apply_forces;
}

void RigidBuoyancy::set_density(float p_density) {
	_density = p_density;
	_set_dirty();
}

float RigidBuoyancy::get_density() const {
	return _density;
}

void RigidBuoyancy::set_com_offset(Vector3 p_com_offset) {
	_com_offset = p_com_offset;
	_set_dirty();
}

Vector3 RigidBuoyancy::get_com_offset() const {
	return _com_offset;
}

void RigidBuoyancy::set_calculate_mass_properties(bool p_calculate) {
	_calculate_mass_properties = p_calculate;
	_set_dirty();
}

bool RigidBuoyancy::get_calculate_mass_properties() const {
	return _calculate_mass_properties;
}

void RigidBuoyancy::set_ignore_waves(bool p_ignore) {
	_ignore_waves = p_ignore;
	_mesh_buoyancy.set_ignore_waves(p_ignore);
	_probe_buoyancy.set_ignore_waves(p_ignore);
}

bool RigidBuoyancy::get_ignore_waves() const {
	return _ignore_waves;
}

void RigidBuoyancy::set_use_buoyancy_scalar(bool p_use) {
	_use_buoyancy_scalar = p_use;
}

bool RigidBuoyancy::get_use_buoyancy_scalar() const {
	return _use_buoyancy_scalar;
}

// Debug
void RigidBuoyancy::set_show_debug(bool p_show) {
	_show_debug = p_show;
	set_debug_mesh_dirty();
}

bool RigidBuoyancy::get_show_debug() const {
	return _show_debug;
}

void RigidBuoyancy::set_debug_color(const Color &p_color) {
	_debug_color = p_color;
	set_debug_mesh_dirty();
}

Color RigidBuoyancy::get_debug_color() const {
	return _debug_color;
}

float RigidBuoyancy::get_submerged_volume() const {
	return _mesh_buoyancy.get_submerged_volume();
}

Vector3 RigidBuoyancy::get_submerged_centroid() const {
	return _mesh_buoyancy.get_submerged_centroid();
}

Vector3 RigidBuoyancy::get_buoyancy_normal() const {
	return _mesh_buoyancy.get_buoyancy_normal();
}

float RigidBuoyancy::get_submerged_ratio() const {
	if (_buoyancy_mode == BUOYANCY_PROBES) {
		return _probe_buoyancy.get_submerged_ratio();
	} else {
		return _mesh_buoyancy.get_submerged_ratio();
	}
}

float RigidBuoyancy::get_volume() const {
	return _mesh_buoyancy.get_mesh_volume();
}

Vector3 RigidBuoyancy::get_centroid() const {
	return _mesh_buoyancy.get_mesh_centroid();
}

float RigidBuoyancy::get_mass() const {
	RigidBody3D *body = Object::cast_to<RigidBody3D>(get_parent());
	return body ? body->get_mass() : -1.0f;
}

Vector3 RigidBuoyancy::get_center_of_mass() const {
	RigidBody3D *body = Object::cast_to<RigidBody3D>(get_parent());
	return body ? body->get_center_of_mass() : Vector3(0, 0, 0);
}

Vector3 RigidBuoyancy::get_inertia() const {
	RigidBody3D *body = Object::cast_to<RigidBody3D>(get_parent());
	return body ? body->get_inertia() : Vector3(0, 0, 0);
}

float RigidBuoyancy::_get_buoyancy_time() const {
	return _buoyancy_time;
}

#pragma endregion

#pragma region Mesh Updates

void RigidBuoyancy::_update_statics() {
	if (_collider == nullptr)
		return;
	if (_buoyancy_mode != BUOYANCY_COLLIDER)
		return;

	// if this is already a convex shape, use it directly
	Ref<ArrayMesh> buoyancy_mesh;
	Ref<ConvexPolygonShape3D> convex_shape = _collider->get_shape();
	if (convex_shape.is_valid()) {
		buoyancy_mesh = convex_shape->get_debug_mesh();

		// otherwise, try to create a simplified convex shape from the existing shape's debug mesh
	} else {
		Ref<Shape3D> shape = _collider->get_shape();
		if (shape.is_valid()) {
			Ref<Mesh> mesh = shape->get_debug_mesh();

			// Use simplified convex shape for buoyancy calculations
			Ref<Shape3D> new_shape = mesh->create_convex_shape(true, true);
			if (new_shape.is_valid()) {
				buoyancy_mesh = Object::cast_to<ConvexPolygonShape3D>(*new_shape)->get_debug_mesh();
			}
		}
	}

	// Delegate to MeshBuoyancy for volume and centroid calculations
	_mesh_buoyancy.set_buoyancy_mesh(buoyancy_mesh);
	_mesh_buoyancy.update_statics(_collider->get_transform());

	if (buoyancy_mesh.is_valid() && buoyancy_mesh->get_surface_count()) {
		// Update mass from collider
		RigidBody3D *body = Object::cast_to<RigidBody3D>(get_parent());
		if (body && _calculate_mass_properties) {
			float mesh_volume = _mesh_buoyancy.get_mesh_volume();
			float mass = mesh_volume * _density;
			body->set_mass(mass);
			body->set_center_of_mass_mode(RigidBody3D::CENTER_OF_MASS_MODE_CUSTOM);
			body->set_center_of_mass(_mesh_buoyancy.get_mesh_centroid() + _com_offset);

			// Calculate inertia tensor
			float ix = 0.0f, iy = 0.0f, iz = 0.0f;
			AABB aabb = buoyancy_mesh->get_aabb();
			// Simplified inertia calculation
			ix = (1.0f / 12.0f) * mass * (aabb.size.y * aabb.size.y + aabb.size.z * aabb.size.z);
			iy = (1.0f / 12.0f) * mass * (aabb.size.x * aabb.size.x + aabb.size.z * aabb.size.z);
			iz = (1.0f / 12.0f) * mass * (aabb.size.x * aabb.size.x + aabb.size.y * aabb.size.y);
			body->set_inertia(Vector3(ix, iy, iz));
		}

		_dirty = false;
	}
}

void RigidBuoyancy::_update_dynamics() {
	if (_collider == nullptr)
		return;
	if (_buoyancy_mode != BUOYANCY_COLLIDER)
		return;

	_mesh_buoyancy.update_dynamics(_collider->get_global_transform(), _collider->get_transform());
	set_debug_mesh_dirty();
}

#pragma endregion

#pragma region Force Application

void RigidBuoyancy::apply_buoyancy_mesh_forces(RigidBody3D *body, float delta) {
	if (!body)
		return;

	ERR_FAIL_COND_MSG(!_buoyancy_material.is_valid(), "Buoyancy material must be valid to apply mesh buoyancy forces.");
	ERR_FAIL_COND_EDMSG(delta <= 0.0f, "Delta time must be positive to apply buoyancy.");
	ERR_FAIL_COND_MSG(_buoyancy_mode != BUOYANCY_COLLIDER, "Buoyancy mode must be set to BUOYANCY_COLLIDER to apply mesh buoyancy forces.");

	const Vector3 gravity = body->get_gravity() * body->get_gravity_scale();

	// Either use the buoyancy scalar or a volume based buoyancy.
	_mesh_buoyancy.set_mass(body->get_mass());
	_mesh_buoyancy.set_buoyancy(_use_buoyancy_scalar ? _buoyancy_material->get_buoyancy() : INFINITY);
	_mesh_buoyancy.update_forces(body->get_global_transform(), gravity);

	// constants
	Vector3 buoyant_force = _mesh_buoyancy.get_buoyancy_force();
	Vector3 submerged_position = _mesh_buoyancy.get_force_position();
	float submerged_volume = _mesh_buoyancy.get_submerged_volume();
	float mesh_volume = _mesh_buoyancy.get_mesh_volume();

	Vector3 submerged_linear_drag = _buoyancy_material->get_linear_drag() * _buoyancy_material->get_linear_drag_scale();
	Vector3 submerged_angular_drag = _buoyancy_material->get_angular_drag() * _buoyancy_material->get_angular_drag_scale();

	if (submerged_volume > 0.0) {
		body->apply_force(buoyant_force, submerged_position - body->get_global_position());

		// Drag is applied axis aligned with the rigid body
		float ratio = submerged_volume / mesh_volume;
		Basis basis = body->get_global_transform().basis.orthonormalized();
		Vector3 one = Vector3(1, 1, 1);

		// Underwater drag. We must get the current state directly from the Physics server.
		Vector3 linear_drag = one - submerged_linear_drag * delta * ratio;
		Vector3 linear_velocity = PhysicsServer3D::get_singleton()->body_get_state(body->get_rid(), PhysicsServer3D::BODY_STATE_LINEAR_VELOCITY);

		Vector3 local_vel = basis.xform_inv(linear_velocity);
		Vector3 global_vel = basis.xform(local_vel * linear_drag);

		body->set_linear_velocity(global_vel);

		Vector3 angular_drag = one - submerged_angular_drag * delta * ratio;
		Vector3 angular_velocity = PhysicsServer3D::get_singleton()->body_get_state(body->get_rid(), PhysicsServer3D::BODY_STATE_ANGULAR_VELOCITY);
		Vector3 local_ang = basis.xform_inv(angular_velocity);
		Vector3 global_ang = basis.xform(local_ang * angular_drag);

		body->set_angular_velocity(global_ang);
	}
}

void RigidBuoyancy::_update_last_probe_transforms() {
	RigidBody3D *body = Object::cast_to<RigidBody3D>(get_parent());
	ERR_FAIL_NULL_MSG(body, "CharacterBuoyancy must be a child of a CharacterBody3D to update submerged state.");

	_probe_buoyancy.update_transforms(body->get_global_transform());
	set_debug_mesh_dirty();
}

void RigidBuoyancy::apply_buoyancy_probe_forces(RigidBody3D *body, float delta) {
	ERR_FAIL_NULL_MSG(body, "CharacterBuoyancy must be a child of a CharacterBody3D to apply buoyancy.");
	ERR_FAIL_COND_MSG(delta <= 0.0f, "Delta time must be positive to apply buoyancy.");

	const float mass = body->get_mass();
	_probe_buoyancy.set_mass(mass);
	ERR_FAIL_COND_MSG(mass <= 0.0f, "Mass must be positive to apply buoyancy.");

	// update forces
	const Vector3 gravity = body->get_gravity() * body->get_gravity_scale();
	_probe_buoyancy.set_buoyancy(_buoyancy_material->get_buoyancy());
	_probe_buoyancy.update_forces(body->get_global_transform(), gravity);

	const auto probes = _probe_buoyancy.get_probes();
	const auto forces = _probe_buoyancy.get_forces();
	ERR_FAIL_COND_MSG(probes.size() != forces.size(), "Probe and forces count mismatch.");

	// nothing to do
	if (probes.size() == 0)
		return;

	// constants
	const Vector3 one = Vector3(1, 1, 1);
	const float probe_ratio = 1.0f / probes.size();
	const Transform3D body_transform = body->get_global_transform();
	const Basis basis = body_transform.basis.orthonormalized();

	Vector3 submerged_linear_drag = _buoyancy_material->get_linear_drag() * _buoyancy_material->get_linear_drag_scale();
	Vector3 submerged_angular_drag = _buoyancy_material->get_angular_drag() * _buoyancy_material->get_angular_drag_scale();

	// add velocity changes from forces
	const int probe_count = probes.size();
	for (int i = 0; i < probe_count; ++i) {
		// don't apply drag if there is no force
		const Vector3 force = forces[i];
		if (force.length()) {
			Vector3 probe = body_transform.xform(probes[i]);
			body->apply_force(force, probe - body_transform.origin);

			// Apply submerged drag per probe
			Vector3 linear_drag = one - submerged_linear_drag * delta * probe_ratio;
			Vector3 linear_velocity = body->get_linear_velocity();
			Vector3 local_vel = basis.xform_inv(linear_velocity);
			Vector3 global_vel = basis.xform(local_vel * linear_drag);
			body->set_linear_velocity(global_vel);

			Vector3 angular_drag = one - submerged_angular_drag * delta * probe_ratio;
			Vector3 angular_velocity = body->get_angular_velocity();
			Vector3 local_ang = basis.xform_inv(angular_velocity);
			Vector3 global_ang = basis.xform(local_ang * angular_drag);
			body->set_angular_velocity(global_ang);
		}
	}
}

#pragma endregion

#pragma region Debug

void RigidBuoyancy::_create_debug_mesh() {
}

// This call has trash performace but it recreates these arrays each frame
void RigidBuoyancy::_update_debug_mesh() {
	if (_debug_mesh_instance == nullptr)
		return;
	if (_debug_mesh.is_valid() == false)
		return;

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
			if (!_collider || !_mesh_buoyancy.get_buoyancy_mesh().is_valid()) {
				return;
			}

			xform = _collider->get_global_transform();

			PackedVector3Array verts = _mesh_buoyancy.get_submerged_verts();
			if (verts.size() == 0) {
				verts = _mesh_buoyancy.get_vertices();
				color = _debug_color.inverted();
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
			if (!body)
				return;

			xform = body->get_global_transform();

			// no probes, skip
			auto probes = _probe_buoyancy.get_probes();
			int probe_count = probes.size();
			if (probe_count == 0)
				return;

			// Show all probes
			for (int idx = 0; idx < probe_count; ++idx) {
				Vector3 probe = probes[idx];

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

			arrays[Mesh::ARRAY_VERTEX] = vertices;
			arrays[Mesh::ARRAY_INDEX] = indices;
		}

		int surf_lines = _debug_mesh->get_surface_count();
		_debug_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_LINES, arrays);

		// disable depth test in game so we can see the mesh
		_debug_material->set_albedo(color);
		if (!Engine::get_singleton()->is_editor_hint()) {
			_debug_material->set_flag(StandardMaterial3D::FLAG_DISABLE_DEPTH_TEST, true);
		}

		_debug_mesh->surface_set_material(surf_lines, _debug_material);

		// origin marker
		_add_marker_surface(get_submerged_centroid(), false);
		_add_marker_surface(get_center_of_mass(), true);

		// try the collider first, and then the parent.
		Node3D *mesh_parent = _collider;
		if (mesh_parent == nullptr)
			mesh_parent = Object::cast_to<Node3D>(get_parent());

		if (mesh_parent) {
			_debug_mesh_instance->set_global_transform(mesh_parent->get_global_transform());
		}
	}
}

void RigidBuoyancy::_destroy_debug_mesh() {
}

#pragma endregion

#pragma region Bindings

void RigidBuoyancy::_bind_methods() {
	// Property bindings
	ClassDB::bind_method(D_METHOD("set_liquid_area", "liquid_area"), &RigidBuoyancy::set_liquid_area);
	ClassDB::bind_method(D_METHOD("get_liquid_area"), &RigidBuoyancy::get_liquid_area);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "liquid_area", PROPERTY_HINT_NODE_TYPE, "LiquidArea"), "set_liquid_area", "get_liquid_area");

	// Buoyancy mode
	ClassDB::bind_method(D_METHOD("set_buoyancy_mode", "buoyancy_mode"), &RigidBuoyancy::set_buoyancy_mode);
	ClassDB::bind_method(D_METHOD("get_buoyancy_mode"), &RigidBuoyancy::get_buoyancy_mode);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "buoyancy_mode", PROPERTY_HINT_ENUM, "BUOYANCY_COLLIDER:0,BUOYANCY_PROBES:1"), "set_buoyancy_mode", "get_buoyancy_mode");

	BIND_ENUM_CONSTANT(BUOYANCY_COLLIDER);
	BIND_ENUM_CONSTANT(BUOYANCY_PROBES);

	ClassDB::bind_method(D_METHOD("set_collider", "collider"), &RigidBuoyancy::set_collider);
	ClassDB::bind_method(D_METHOD("get_collider"), &RigidBuoyancy::get_collider);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "collider", PROPERTY_HINT_NODE_TYPE, "CollisionShape3D"), "set_collider", "get_collider");

	ClassDB::bind_method(D_METHOD("set_buoyancy_material", "buoyancy_material"), &RigidBuoyancy::set_buoyancy_material);
	ClassDB::bind_method(D_METHOD("get_buoyancy_material"), &RigidBuoyancy::get_buoyancy_material);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "buoyancy_material", PROPERTY_HINT_RESOURCE_TYPE, "BuoyancyMaterial"), "set_buoyancy_material", "get_buoyancy_material");

	// Buoyancy probes
	ClassDB::bind_method(D_METHOD("set_probes", "buoyancy_probes"), &RigidBuoyancy::set_probes);
	ClassDB::bind_method(D_METHOD("get_probes"), &RigidBuoyancy::get_probes);
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_VECTOR3_ARRAY, "probes"), "set_probes", "get_probes");

	// ---
	ADD_GROUP("Physics", "");

	ClassDB::bind_method(D_METHOD("set_apply_forces", "apply_forces"), &RigidBuoyancy::set_apply_forces);
	ClassDB::bind_method(D_METHOD("get_apply_forces"), &RigidBuoyancy::get_apply_forces);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "apply_forces"), "set_apply_forces", "get_apply_forces");

	ClassDB::bind_method(D_METHOD("set_ignore_waves", "ignore_waves"), &RigidBuoyancy::set_ignore_waves);
	ClassDB::bind_method(D_METHOD("get_ignore_waves"), &RigidBuoyancy::get_ignore_waves);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "ignore_waves"), "set_ignore_waves", "get_ignore_waves");

	ClassDB::bind_method(D_METHOD("set_use_buoyancy_scalar", "use_buoyancy_scalar"), &RigidBuoyancy::set_use_buoyancy_scalar);
	ClassDB::bind_method(D_METHOD("get_use_buoyancy_scalar"), &RigidBuoyancy::get_use_buoyancy_scalar);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "use_buoyancy_scalar"), "set_use_buoyancy_scalar", "get_use_buoyancy_scalar");

	// Read only
	ClassDB::bind_method(D_METHOD("get_submerged_volume"), &RigidBuoyancy::get_submerged_volume);
	ClassDB::bind_method(D_METHOD("get_submerged_centroid"), &RigidBuoyancy::get_submerged_centroid);
	ClassDB::bind_method(D_METHOD("get_buoyancy_normal"), &RigidBuoyancy::get_buoyancy_normal);
	ClassDB::bind_method(D_METHOD("get_submerged_ratio"), &RigidBuoyancy::get_submerged_ratio);

	ClassDB::bind_method(D_METHOD("get_volume"), &RigidBuoyancy::get_volume);
	ClassDB::bind_method(D_METHOD("get_centroid"), &RigidBuoyancy::get_centroid);

	// Info properties (read-only)
	ClassDB::bind_method(D_METHOD("get_mass"), &RigidBuoyancy::get_mass);
	ClassDB::bind_method(D_METHOD("get_center_of_mass"), &RigidBuoyancy::get_center_of_mass);
	ClassDB::bind_method(D_METHOD("get_inertia"), &RigidBuoyancy::get_inertia);

	ClassDB::bind_method(D_METHOD("_get_buoyancy_time"), &RigidBuoyancy::_get_buoyancy_time);

	// Function calls
	ClassDB::bind_method(D_METHOD("apply_buoyancy_mesh_forces"), &RigidBuoyancy::apply_buoyancy_mesh_forces);
	ClassDB::bind_method(D_METHOD("apply_buoyancy_probe_forces"), &RigidBuoyancy::apply_buoyancy_probe_forces);

	// ---
	ADD_GROUP("Mass Properties", "");

	ClassDB::bind_method(D_METHOD("set_calculate_mass_properties", "calculate_mass_properties"), &RigidBuoyancy::set_calculate_mass_properties);
	ClassDB::bind_method(D_METHOD("get_calculate_mass_properties"), &RigidBuoyancy::get_calculate_mass_properties);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "calculate_mass_properties"), "set_calculate_mass_properties", "get_calculate_mass_properties");

	ClassDB::bind_method(D_METHOD("set_density", "density"), &RigidBuoyancy::set_density);
	ClassDB::bind_method(D_METHOD("get_density"), &RigidBuoyancy::get_density);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "density", PROPERTY_HINT_RANGE, "1,10000,1,suffix:kg*m^3"), "set_density", "get_density");

	ClassDB::bind_method(D_METHOD("set_com_offset", "com_offset"), &RigidBuoyancy::set_com_offset);
	ClassDB::bind_method(D_METHOD("get_com_offset"), &RigidBuoyancy::get_com_offset);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "com_offset"), "set_com_offset", "get_com_offset");

	// ---
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "volume", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_READ_ONLY | PROPERTY_USAGE_NO_INSTANCE_STATE | PROPERTY_USAGE_EDITOR), "", "get_volume");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "centroid", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_READ_ONLY | PROPERTY_USAGE_NO_INSTANCE_STATE | PROPERTY_USAGE_EDITOR), "", "get_centroid");

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "body_mass", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_READ_ONLY | PROPERTY_USAGE_NO_INSTANCE_STATE | PROPERTY_USAGE_EDITOR), "", "get_mass");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "body_center_of_mass", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_READ_ONLY | PROPERTY_USAGE_NO_INSTANCE_STATE | PROPERTY_USAGE_EDITOR), "", "get_center_of_mass");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "body_inertia", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_READ_ONLY | PROPERTY_USAGE_NO_INSTANCE_STATE | PROPERTY_USAGE_EDITOR), "", "get_inertia");

	// Debug
	ADD_GROUP("Debug", "");
	ClassDB::bind_method(D_METHOD("set_show_debug", "show_debug"), &RigidBuoyancy::set_show_debug);
	ClassDB::bind_method(D_METHOD("get_show_debug"), &RigidBuoyancy::get_show_debug);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "show_debug"), "set_show_debug", "get_show_debug");

	ClassDB::bind_method(D_METHOD("set_debug_color", "debug_color"), &RigidBuoyancy::set_debug_color);
	ClassDB::bind_method(D_METHOD("get_debug_color"), &RigidBuoyancy::get_debug_color);
	ADD_PROPERTY(PropertyInfo(Variant::COLOR, "debug_color"), "set_debug_color", "get_debug_color");

	ADD_SIGNAL(MethodInfo("submerged_changed"));
}

#pragma endregion
