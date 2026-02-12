#include "rope.h"
#include "liquid_area.h"
#include "rope_anchors_base.h"
#include "rope_appearance.h"
#include "rope_attachments_base.h"
#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/editor_interface.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/classes/physics_direct_space_state3d.hpp>
#include <godot_cpp/classes/physics_ray_query_parameters3d.hpp>
#include <godot_cpp/classes/physics_server3d.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/sphere_shape3d.hpp>
#include <godot_cpp/classes/sub_viewport.hpp>
#include <godot_cpp/classes/viewport.hpp>
#include <godot_cpp/classes/world3d.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include "halyard_utils.h"

// dump initial conditions for debugging stretch grow directions.
#define DEBUG_INITIAL_POS false

// Basis indexes
const int X = 0;
const int Y = 1;
const int Z = 2;

Rope::Rope() {
	_rope_mesh.instantiate();
	set_base(_rope_mesh->get_rid());

	auto ps = PhysicsServer3D::get_singleton();
	_physics_body = ps->body_create();
	ps->body_set_mode(_physics_body, PhysicsServer3D::BODY_MODE_STATIC);

	// Initialize cached objects for reuse
	_ray_cast.instantiate();
	_exclusion_list.append(_physics_body); // exclude self
	_ray_cast->set_exclude(_exclusion_list);
	_ray_cast->set_collide_with_areas(false);
	_ray_cast->set_collide_with_bodies(true);
}

Rope::Rope(const Rope &other) {
}

Rope::~Rope() {
	if (_appearance != nullptr)
		_appearance->disconnect("changed", Callable(this, "_on_appearance_changed"));

	_clear_instances();

	PhysicsServer3D::get_singleton()->free_rid(_physics_body);
}

void Rope::_bind_methods() {
	BIND_ENUM_CONSTANT(Start);
	BIND_ENUM_CONSTANT(End);

	ClassDB::bind_method(D_METHOD("get_baked_mesh"), &Rope::get_baked_mesh);
	ClassDB::bind_method(D_METHOD("get_current_rope_length"), &Rope::get_current_rope_length);
	ClassDB::bind_method(D_METHOD("get_particle_count_for_length"), &Rope::get_particle_count_for_length);

	ClassDB::bind_method(D_METHOD("get_rope_frame_count"), &Rope::get_rope_frame_count);
	ClassDB::bind_method(D_METHOD("get_rope_frame", "index"), &Rope::get_rope_frame);
	ClassDB::bind_method(D_METHOD("get_all_rope_frames"), &Rope::get_all_rope_frames);

	ClassDB::bind_method(D_METHOD("get_particle_positions"), &Rope::get_particle_positions);

	ClassDB::bind_method(D_METHOD("get_link_count"), &Rope::get_rope_frame_count);
	ClassDB::bind_method(D_METHOD("get_link", "index"), &Rope::get_rope_frame);
	ClassDB::bind_method(D_METHOD("get_all_links"), &Rope::get_all_rope_frames);

	// virtuals
	GDVIRTUAL_BIND(_get_anchor_count)
	GDVIRTUAL_BIND(_get_anchor_position, "idx")
	GDVIRTUAL_BIND(_get_anchor_transform, "idx")

	GDVIRTUAL_BIND(_get_attachment_count)
	GDVIRTUAL_BIND(_get_attachment_position, "idx")
	GDVIRTUAL_BIND(_get_attachment_nodepath, "idx")
	GDVIRTUAL_BIND(_get_attachment_transform, "idx")

	// pass through
	ClassDB::bind_method(D_METHOD("get_rope_width"), &Rope::get_rope_width);
	ClassDB::bind_method(D_METHOD("get_rope_sides"), &Rope::get_rope_sides);
	ClassDB::bind_method(D_METHOD("get_rope_twist"), &Rope::get_rope_twist);
	ClassDB::bind_method(D_METHOD("get_rope_lod"), &Rope::get_rope_lod);
	ClassDB::bind_method(D_METHOD("get_start_attachment"), &Rope::get_start_attachment);
	ClassDB::bind_method(D_METHOD("get_start_offset"), &Rope::get_start_offset);
	ClassDB::bind_method(D_METHOD("get_end_attachment"), &Rope::get_end_attachment);
	ClassDB::bind_method(D_METHOD("get_end_offset"), &Rope::get_end_offset);
	ClassDB::bind_method(D_METHOD("get_particles_per_meter"), &Rope::get_particles_per_meter);
	ClassDB::bind_method(D_METHOD("get_attachments"), &Rope::get_attachments);

	ClassDB::bind_method(D_METHOD("_on_appearance_changed"), &Rope::_on_appearance_changed);

	EXPORT_PROPERTY(Variant::FLOAT, rope_length, Rope);
	EXPORT_PROPERTY_ENUM(grow_from, "Start,End", Rope);
	EXPORT_PROPERTY(Variant::BOOL, jitter_initial_position, Rope);

	EXPORT_PROPERTY(Variant::NODE_PATH, start_anchor, Rope);
	EXPORT_PROPERTY(Variant::NODE_PATH, end_anchor, Rope);
	ClassDB::bind_method(D_METHOD("set_anchors", "anchors"), &Rope::set_anchors);
	ClassDB::bind_method(D_METHOD("get_anchors"), &Rope::get_anchors);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "anchors", PROPERTY_HINT_RESOURCE_TYPE, "RopeAnchorsBase"), "set_anchors", "get_anchors");

	ClassDB::bind_method(D_METHOD("set_appearance", "appearance"), &Rope::set_appearance);
	ClassDB::bind_method(D_METHOD("get_appearance"), &Rope::get_appearance);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "appearance", PROPERTY_HINT_RESOURCE_TYPE, "RopeAppearance"), "set_appearance", "get_appearance");

	// simulation parameters
	ADD_GROUP("Simulation", "");
	EXPORT_PROPERTY(Variant::BOOL, simulate, Rope);
	EXPORT_PROPERTY(Variant::FLOAT, preprocess_time, Rope);
	EXPORT_PROPERTY(Variant::FLOAT, simulation_rate, Rope);
	EXPORT_PROPERTY_RANGED(Variant::INT, stiffness_iterations, Rope, "1,50,1,hide_slider");
	EXPORT_PROPERTY_RANGED(Variant::FLOAT, stiffness, Rope, "0.01,1.99,0.01");
	EXPORT_PROPERTY_RANGED(Variant::FLOAT, friction, Rope, "0.0,1.0,0.01");

	ClassDB::bind_method(D_METHOD("set_collision_layer", "collision_layer"), &Rope::set_collision_layer);
	ClassDB::bind_method(D_METHOD("get_collision_layer"), &Rope::get_collision_layer);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_layer", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_layer", "get_collision_layer");

	ClassDB::bind_method(D_METHOD("set_collision_mask", "collision_mask"), &Rope::set_collision_mask);
	ClassDB::bind_method(D_METHOD("get_collision_mask"), &Rope::get_collision_mask);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_mask", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_mask", "get_collision_mask");

	// forces
	ADD_GROUP("Forces", "");

	// Buoyancy
	EXPORT_PROPERTY(Variant::BOOL, apply_buoyancy, Rope);
	EXPORT_PROPERTY(Variant::FLOAT, buoyancy_scale, Rope);
	EXPORT_PROPERTY(Variant::FLOAT, submerged_drag, Rope);
	ClassDB::bind_method(D_METHOD("set_liquid_area", "liquid_area"), &Rope::set_liquid_area);
	ClassDB::bind_method(D_METHOD("get_liquid_area"), &Rope::get_liquid_area);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "liquid_area", PROPERTY_HINT_NODE_TYPE, "LiquidArea"), "set_liquid_area", "get_liquid_area");

	// Wind
	EXPORT_PROPERTY(Variant::BOOL, apply_wind, Rope);
	EXPORT_PROPERTY(Variant::FLOAT, wind_scale, Rope);
	EXPORT_PROPERTY(Variant::VECTOR3, wind, Rope);

	ClassDB::bind_method(D_METHOD("set_wind_noise", "wind_noise"), &Rope::set_wind_noise);
	ClassDB::bind_method(D_METHOD("get_wind_noise"), &Rope::get_wind_noise);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "wind_noise", PROPERTY_HINT_RESOURCE_TYPE, "FastNoiseLite"), "set_wind_noise", "get_wind_noise");

	// Gravity
	EXPORT_PROPERTY(Variant::BOOL, apply_gravity, Rope);
	EXPORT_PROPERTY(Variant::VECTOR3, gravity, Rope);
	EXPORT_PROPERTY(Variant::FLOAT, gravity_scale, Rope);

	// Damping
	EXPORT_PROPERTY(Variant::BOOL, apply_damping, Rope);
	EXPORT_PROPERTY(Variant::FLOAT, damping_factor, Rope);
}

#pragma region Lifecycle

void Rope::_notification(int p_what) {
	double delta = Engine::get_singleton()->is_in_physics_frame() ? get_physics_process_delta_time() : get_process_delta_time();
	bool visible = is_visible_in_tree();

	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {
			if (Engine::get_singleton()->is_editor_hint() == false) {
				if (_liquid_area == nullptr) {
					SceneTree *tree = get_tree();
					_liquid_area = LiquidArea::get_liquid_area(tree);
				}
			}
		} break;

		case NOTIFICATION_ENTER_WORLD: {
			Ref<World3D> w3d = get_world_3d();
			auto ps = PhysicsServer3D::get_singleton();
			auto rs = RenderingServer::get_singleton();
			ERR_FAIL_NULL(w3d);

			if (w3d.is_valid() && rs && ps) {
				RID space = w3d->get_space();
				ps->body_set_space(_physics_body, space);

				RID scenario = w3d->get_scenario();
				for (auto instance : _instances) {
					rs->instance_set_scenario(instance, scenario);
					rs->instance_set_visible(instance, visible);
				}
			}

		} break;
		case NOTIFICATION_EXIT_WORLD: {
			auto ps = PhysicsServer3D::get_singleton();
			auto rs = RenderingServer::get_singleton();
			if (ps && rs) {
				ps->body_set_space(_physics_body, RID());
				for (auto instance : _instances) {
					rs->instance_set_scenario(instance, RID());
					rs->instance_attach_skeleton(instance, RID());
					rs->instance_set_visible(instance, visible);
				}
			}

		} break;

		case NOTIFICATION_TRANSFORM_CHANGED: {
		} break;

		case NOTIFICATION_VISIBILITY_CHANGED: {
			_set_instances_visible(visible);
		} break;

			// case NOTIFICATION_DISABLED: {
			// 	if (is_inside_tree() && (disable_mode == DISABLE_MODE_REMOVE)) {
			// 		_prepare_physics_server();
			// 	}
			// } break;

			// case NOTIFICATION_ENABLED: {
			// 	if (is_inside_tree() && (disable_mode == DISABLE_MODE_REMOVE)) {
			// 		_prepare_physics_server();
			// 	}
			// } break;

		case NOTIFICATION_READY: {
			_internal_ready();
		} break;

		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
			// SCOPED_TIMER(Rope_INTERNAL_PHYSICS_PROCESS);
			_internal_physics_process(delta);
		} break;

		case NOTIFICATION_INTERNAL_PROCESS: {
			// SCOPED_TIMER(Rope_INTERNAL_PROCESS);
			_internal_process(delta);
		} break;

		case NOTIFICATION_EXIT_TREE: {
			_liquid_area = nullptr;
		} break;
	}
}

void Rope::_internal_ready(void) {
	set_process_internal(true);
	set_physics_process_internal(true);
	_queue_redraw();
}

void Rope::_internal_process(double delta) {
	if (_pop_is_dirty()) {
		// cancel out the rotation for our transform.
		// the simulation can't work with it.
		set_global_transform(Transform3D(Basis(), get_global_position()));

		_draw_rope();
	}
}

void Rope::_internal_physics_process(double delta) {
	_time += delta;
	_simulation_delta += delta;

	auto simulation_step = 1.0 / float(_simulation_rate);
	if (_simulation_delta < simulation_step)
		return;

	if (_pop_rebuild()) {
		_rebuild_rope();
	}

	if (_particles.size() >= 2) {
		// unattach all the particles
		for (Particle &p : _particles)
			p.attached = false;

		// set the anchor point for the beginning
		// run simulation
		_update_anchors();
		if (_simulate) {
			_update_physics(float(simulation_step), 1);
			_queue_redraw();
		}
	}

	// reset delta when we simulate
	_simulation_delta = 0.0;
}

void Rope::_set_instances_visible(bool p_visible) {
	auto rs = RenderingServer::get_singleton();
	ERR_FAIL_NULL_MSG(rs, "RenderingServer missing");

	for (auto &rid : _instances) {
		rs->instance_set_visible(rid, p_visible);
	}
}

void Rope::_clear_instances() {
	auto rs = RenderingServer::get_singleton();
	ERR_FAIL_NULL_MSG(rs, "RenderingServer missing");

	for (auto &rid : _instances) {
		rs->free_rid(rid);
	}
	_instances.clear();
}

void Rope::_rebuild_instances() {
	auto rs = RenderingServer::get_singleton();
	Ref<World3D> w3d = get_world_3d();
	ERR_FAIL_NULL_MSG(rs, "RenderingServer missing");
	ERR_FAIL_NULL_MSG(w3d, "World3D missing");

	if (rs && w3d.is_valid()) {
		// only rebuild instances if we have an appearance
		if (_appearance.is_valid()) {
			auto scenario = w3d->get_scenario();
			auto mesh = _appearance->get_array_mesh();
			if (scenario.is_valid() && mesh.is_valid()) {
				// create one instance for each gap between particles
				// this will be one less than the particle count
				auto count = _particles.size() - 1;
				for (int idx = 0; idx < count; idx++) {
					RID instance = rs->instance_create2(mesh->get_rid(), scenario);
					if (instance.is_valid()) {
						_instances.push_back(instance);
						rs->instance_attach_object_instance_id(instance, get_instance_id());
						rs->instance_set_visible(instance, is_visible_in_tree());
					}
				}
			}
		}
	}
}

Vector3 _small_offset(float d) {
	float x = rand() % 2 ? -1.0 : 1.0;
	float y = rand() % 2 ? -1.0 : 1.0;
	float z = rand() % 2 ? -1.0 : 1.0;
	return Vector3(x * d, y * d, z * d);
}

void Rope::_rebuild_rope() {
	// free previous shapes
	_clear_physics_shapes();
	_clear_instances();
	_frames.clear();

	auto global_position = get_global_position();
	const int previous_count = _particles.size();
	int particle_count = get_particle_count_for_length();

	// calculate our direction and previous position based upon
	// the end we're expanding from
	Vector3 direction = _gravity.normalized();
	Vector3 current_pos = global_position;
	if (previous_count >= 2) {
		if (_grow_from == Start) {
			current_pos = _particles[0].pos_cur;
			direction = current_pos.direction_to(_particles[1].pos_cur);
		} else {
			current_pos = _particles[previous_count - 1].pos_cur;
			direction = current_pos.direction_to(_particles[previous_count - 2].pos_cur);
		}
	} else {
		Vector3 start = global_position;
		Vector3 end = global_position;
		Transform3D xform;
		if (_grow_from == Start) {
			if (_get_node_transform(_start_anchor, xform))
				start = xform.origin;
			if (_get_node_transform(_end_anchor, xform))
				end = xform.origin;
		} else {
			if (_get_node_transform(_start_anchor, xform))
				end = xform.origin;
			if (_get_node_transform(_end_anchor, xform))
				start = xform.origin;
		}
		current_pos = start;
		direction = (end - start).normalized();
		if (direction.length() == 0)
			direction = _gravity.normalized();
	}

	// grow
	auto segment_length = _rope_length / (particle_count - 1);
	float jitter_offset = segment_length / 10.0;
	Vector3 direction_segment = direction * segment_length;
	int insert_idx = 0;
	while (_particles.size() < particle_count) {
		Particle particle;
		// NOTE: this flag generates a small initial offset to the calculated previous position
		// so the simulation isn't "perfectly aligned" which can cause problems growing the rope
		// in a direction perfectly aligned with the gravity vector.
		//
		// If the row grows too slowly "stacking" can still occur as the stiffness will stablize it.
		Vector3 jitter_prev = (_jitter_initial_position ? _small_offset(jitter_offset) : Vector3());
		Vector3 jitter_cur = (_jitter_initial_position ? _small_offset(jitter_offset) : Vector3());
		particle.pos_prev = current_pos + jitter_prev;
		particle.pos_cur = current_pos + jitter_cur;
		particle.attached = false;

		// for next particle
		auto jitter_dir = (direction + (_jitter_initial_position ? _small_offset(0.1) : Vector3())).normalized();
		current_pos = current_pos + direction_segment;

		if (_grow_from == Start)
			_particles.insert(insert_idx++, particle);
		else
			_particles.push_back(particle);

		_stiff_rope(1);
	}

#if DEBUG_INITIAL_POS
	print_line("Rebuild Rope:");
	for (auto &p : _particles) {
		print_line(p.pos_cur);
	}
#endif

	// shrink
	if (particle_count > 0) {
		while (_particles.size() > particle_count) {
			if (_grow_from == Start)
				_particles.remove_at(0);
			else
				_particles.remove_at(_particles.size() - 1);
		}
	}

	_clear_physics_shapes();
	_rebuild_physics_shapes();
	_rebuild_instances();
	_update_anchors();

	// only run preprocess on the first build
	if (previous_count == 0) {
		const float preprocess_delta = 1.0 / _simulation_rate;
		const int preprocess_iterations = Math::max(int(_simulation_rate * _preprocess_time), 1);
		_update_physics(preprocess_delta, preprocess_iterations);
	}

	_compute_particle_normals();
	_queue_redraw();
}

void Rope::_queue_rope_rebuild() {
	_rebuild = true;
}

bool Rope::_pop_rebuild() {
	bool should_rebuild = _rebuild;
	_rebuild = false;
	return should_rebuild;
}

#pragma endregion

#pragma region Accessors

// appearance passthrough accessors
#define APPEARENCE_ACCESSOR(m_type, m_name, m_default) \
	m_type Rope::get_##m_name() const { return _appearance != nullptr ? _appearance->get_##m_name() : m_default; }
APPEARENCE_ACCESSOR(float, rope_width, 0.125)
APPEARENCE_ACCESSOR(float, rope_twist, 1.0)
APPEARENCE_ACCESSOR(int, rope_lod, 2)
APPEARENCE_ACCESSOR(Ref<Material>, material, nullptr)
APPEARENCE_ACCESSOR(NodePath, start_attachment, NodePath())
APPEARENCE_ACCESSOR(float, start_offset, 0.0)
APPEARENCE_ACCESSOR(NodePath, end_attachment, NodePath())
APPEARENCE_ACCESSOR(float, end_offset, 0.0)
APPEARENCE_ACCESSOR(Ref<RopeAttachmentsBase>, attachments, nullptr)
APPEARENCE_ACCESSOR(float, particles_per_meter, 2.0)
#undef APPEARENCE_ACCESSOR

void Rope::_on_appearance_changed() {
	if (is_inside_tree())
		_queue_rope_rebuild();
}

Ref<RopeAppearance> Rope::get_appearance() const {
	return _appearance;
}

void Rope::set_appearance(const Ref<RopeAppearance> &val) {
	if (_appearance != val) {
		// disconnect old
		if (_appearance != nullptr) {
			_appearance->disconnect("changed", Callable(this, "_on_appearance_changed"));
		}

		_appearance = val;

		// connect new
		if (_appearance != nullptr) {
			_appearance->connect("changed", Callable(this, "_on_appearance_changed"));
		}

		_queue_redraw();
	}
}

uint64_t Rope::get_particle_count() const {
	return _particles.size();
}

Ref<ArrayMesh> Rope::get_baked_mesh() const {
	if (_rope_mesh.is_valid()) {
		_rope_mesh->_update_mesh_internal(_frames, get_material());
		return _rope_mesh;
	}

	return Ref<ArrayMesh>();
}

void Rope::set_liquid_area(LiquidArea *liquid_area) {
	_liquid_area = liquid_area;
}

LiquidArea *Rope::get_liquid_area() const {
	return _liquid_area;
}

float Rope::get_current_rope_length() const {
	float length = 0.0;
	for (int i = 0; i < _particles.size() - 1; i++)
		length += (_particles[i + 1].pos_cur - _particles[i].pos_cur).length();
	return length;
}

int Rope::get_rope_sides() const {
	if (_appearance == nullptr)
		return 6;
	else {
		auto sides = _appearance->get_rope_sides();
		if (sides >= 3 && sides <= MAX_ROPE_SIDES) {
			return sides;
		} else {
			auto diameter = get_rope_width();
			const float ratio = UtilityFunctions::ease(diameter, .2);
			return Math::clamp(int(Math::lerp(3, 12, ratio)), 3, 12);
		}
	}
}

int Rope::get_link_count() const {
	return _links.size();
}

Transform3D Rope::get_link(int index) const {
	if (index < 0 || index >= _links.size())
		return Transform3D();

	return _links[index];
}

TypedArray<Transform3D> Rope::get_all_links() const {
	TypedArray<Transform3D> array;
	for (const auto &link : _links)
		array.push_back(link);
	return array;
}

int Rope::get_rope_frame_count() const {
	return _frames.size();
}

Transform3D Rope::get_rope_frame(int index) const {
	if (index < 0 || index >= _frames.size())
		return Transform3D();

	return _frames[index];
}

TypedArray<Transform3D> Rope::get_all_rope_frames() const {
	TypedArray<Transform3D> array;
	for (const auto &frame : _frames)
		array.push_back(frame);
	return array;
}

TypedArray<Vector3> Rope::get_particle_positions() const {
	TypedArray<Vector3> array;

	// manual copy
	for (const auto &particle : _particles)
		array.push_back(particle.pos_cur);
	return array;
}

uint64_t Rope::get_particle_count_for_length() const {
	int particle_count = uint64_t(get_rope_length() * get_particles_per_meter()) + 1;
	particle_count = Math::max(particle_count, 2);
	return particle_count;
}

#pragma endregion

#pragma region Subclassing

int Rope::_get_anchor_count() const {
	if (GDVIRTUAL_IS_OVERRIDDEN(_get_anchor_count)) {
		int ret_val;
		GDVIRTUAL_CALL(_get_anchor_count, ret_val);
		return ret_val;
	}

	// default implementation
	if (_anchors.is_valid())
		return _anchors->get_count(this);
	return 0;
}

int Rope::_get_attachment_count() const {
	if (GDVIRTUAL_IS_OVERRIDDEN(_get_attachment_count)) {
		int ret_val;
		GDVIRTUAL_CALL(_get_attachment_count, ret_val);
		return ret_val;
	}

	// default implementation
	if (_appearance.is_valid()) {
		if (_appearance->_attachments.is_valid())
			return _appearance->_attachments->get_count(this);
	}
	return 0;
}

Transform3D Rope::_get_anchor_transform(int idx) const {
	if (GDVIRTUAL_IS_OVERRIDDEN(_get_anchor_transform)) {
		Transform3D ret_val;
		GDVIRTUAL_CALL(_get_anchor_transform, idx, ret_val);
		return ret_val;
	}

	// default implementation
	if (_anchors.is_valid()) {
		return _anchors->get_transform(idx, this);
	}
	return Transform3D();
}

float Rope::_get_anchor_position(int idx) const {
	if (GDVIRTUAL_IS_OVERRIDDEN(_get_anchor_position)) {
		float ret_val;
		GDVIRTUAL_CALL(_get_anchor_position, idx, ret_val);
		return ret_val;
	}

	// default implementation
	if (_anchors.is_valid()) {
		float position = _anchors->get_position(idx, this);
		return position;
	}
	return 0.0;
}

float Rope::_get_attachment_position(int idx) const {
	if (GDVIRTUAL_IS_OVERRIDDEN(_get_attachment_position)) {
		float ret_val;
		GDVIRTUAL_CALL(_get_attachment_position, idx, ret_val);
		return ret_val;
	}

	// default implementation
	if (_appearance.is_valid()) {
		if (_appearance->_attachments.is_valid()) {
			return _appearance->_attachments->get_position(idx, this);
		}
	}
	return 0.0;
}

NodePath Rope::_get_attachment_nodepath(int idx) const {
	if (GDVIRTUAL_IS_OVERRIDDEN(_get_attachment_nodepath)) {
		NodePath ret_val;
		GDVIRTUAL_CALL(_get_attachment_nodepath, idx, ret_val);
		return ret_val;
	}

	// default implementation
	if (_appearance.is_valid()) {
		if (_appearance->_attachments.is_valid()) {
			return _appearance->_attachments->get_nodepath(idx, this);
		}
	}
	return NodePath();
}

Transform3D Rope::_get_attachment_transform(int idx) const {
	if (GDVIRTUAL_IS_OVERRIDDEN(_get_attachment_transform)) {
		Transform3D ret_val;
		GDVIRTUAL_CALL(_get_attachment_transform, idx, ret_val);
		return ret_val;
	}

	// default implementation
	return Transform3D();
}

#pragma endregion

#pragma region Anchors & Attachments

int Rope::_get_index_for_position(float position) const {
	int last = _particles.size() - 1;
	return Math::clamp(int(last * position), 0, last);
}

bool Rope::_get_node_transform(const NodePath &path, Transform3D &xform) const {
	Node3D *node = cast_to<Node3D>(get_node_or_null(path));
	if (node && node->is_visible()) {
		xform = node->get_global_transform();
		return true;
	}

	return false;
}

void Rope::_update_anchor(NodePath &anchor, float position) {
	Transform3D xform;
	if (_get_node_transform(anchor, xform)) {
		int index = _get_index_for_position(position);
		_particles[index].pos_cur = xform.origin;
		_particles[index].pos_prev = xform.origin;
		_particles[index].attached = true;
	}
}

void Rope::_update_anchors() {
	// for the first anchor, move the whole rope instead of the point
	Transform3D xform;
	if (_get_node_transform(_start_anchor, xform)) {
		set_global_position(xform.origin);
	}

	// clear previous position attachments
	for (auto &particle : _particles)
		particle.attached = false;

	// mark the mid anchors
	int count = _get_anchor_count();
	for (int idx = 0; idx < count; idx++) {
		auto position = _get_anchor_position(idx);
		if (position != -1) {
			auto xform = _get_anchor_transform(idx);
			int index = _get_index_for_position(position);

			_particles[index].pos_cur = xform.origin;
			_particles[index].pos_prev = xform.origin;
			_particles[index].attached = true;
		}
	}

	// attach to the start and end anchors last so that they are always
	// anchored and not stomped on by mid anchors above.
	_update_anchor(_start_anchor, 0.0);
	_update_anchor(_end_anchor, 1.0);
}

void Rope::set_anchors(const Ref<RopeAnchorsBase> &val) {
	_anchors = val;
	_queue_redraw();
}

Ref<RopeAnchorsBase> Rope::get_anchors() const {
	return _anchors;
}

#pragma endregion

#pragma region Frame Calculations

float Rope::_get_average_segment_length() const {
	// segment count is N-1 particle count
	return get_rope_length() / float(_particles.size() - 1);
}

PackedVector3Array Rope::_get_control_points_for_particle(int index) const {
	auto segment_length = _get_average_segment_length();
	const auto last_index = _particles.size() - 1;
	const auto projected = (_particles[index].T * segment_length);

	PackedVector3Array p;
	p.resize(4);

	// at index 0 project a fictitious previous point
	if (index > 0)
		p[0] = _particles[index - 1].pos_cur;
	if (index == 0)
		p[0] = _particles[index].pos_cur - projected;

	// p[1] is the current point at index
	p[1] = _particles[index].pos_cur;

	if (index < last_index)
		p[2] = _particles[index + 1].pos_cur;
	else
		p[2] = _particles[index].pos_cur + projected;

	// last
	if (index < last_index - 1)
		p[3] = _particles[index + 2].pos_cur;
	else if (index < last_index)
		p[3] = _particles[index + 1].pos_cur + projected;
	else
		p[3] = _particles[index].pos_cur + projected * 2.0;
	return p;
}

Pair<Vector3, Vector3> Rope::_catmull_interpolate(const Vector3 &p0, const Vector3 &p1, const Vector3 &p2, const Vector3 &p3, float tension, float t) const {
	float t_sqr = t * t;
	float t_cube = t_sqr * t;

	Vector3 m1 = (1.0 - tension) / 2.0 * (p2 - p0);
	Vector3 m2 = (1.0 - tension) / 2.0 * (p3 - p1);

	Vector3 a = (2.0 * (p1 - p2)) + m1 + m2;
	Vector3 b = (-3.0 * (p1 - p2)) - (2.0 * m1) - m2;

	Vector3 point = (a * t_cube) + (b * t_sqr) + (m1 * t) + p1;
	Vector3 tangent = ((3.0 * a * t_sqr) + (2.0 * b * t) + m1).normalized();

	return { point, tangent };
}

void Rope::_compute_particle_normals() {
	if (_particles.size()) {
		auto origin = get_global_position();

		auto start = _particles[0];
		start.T = (_particles[1].pos_cur - start.pos_cur).normalized();
		start.N = (start.pos_cur - origin).normalized();
		start.B = start.N.cross(start.T).normalized();
		_particles[0] = start;

		auto end_index = _particles.size() - 1;
		auto end = _particles[end_index];
		end.T = (end.pos_cur - _particles[end_index - 1].pos_cur).normalized();
		end.N = (end.pos_cur - origin).normalized();
		end.B = end.N.cross(end.T).normalized();
		_particles[end_index] = end;

		for (int i = 1; i < _particles.size() - 1; i++) {
			auto particle = _particles[i];
			particle.T = (_particles[i + 1].pos_cur - _particles[i - 1].pos_cur).normalized();
			particle.N = (particle.pos_cur - origin).normalized();
			particle.B = particle.N.cross(particle.T).normalized();
			_particles[i] = particle;
		}
	}
}

// compute a stable normal/binormal frame using parallel transport to reduce twisting
// ref: https://en.wikipedia.org/wiki/Parallel_transport
void Rope::_compute_parallel_transport(LocalVector<Transform3D> &frames) const {
	auto count = frames.size();
	if (count > 0) {
		// initial tangent
		Vector3 T = frames[0].basis.get_column(Y).normalized();

		// // use the basis for our rope start aligned to the first tangent
		// // as the reference frame. this keeps the normals and U coordinate
		// // from swinging wildly when the curvature changes orientation
		Basis global_basis = get_global_basis();
		auto basis_y = -global_basis.get_column(Y);
		global_basis.rotate_to_align(basis_y, T);
		Vector3 ref = global_basis.get_column(Y);

		// FIXME: this will spin wildly when tilted 90 off x & y axis.

		Vector3 N = global_basis.get_column(X);
		Vector3 B = global_basis.get_column(Z);

		frames[0].basis.set_column(X, N);
		frames[0].basis.set_column(Z, B);

		// parallel transport subsequent normals
		for (int i = 1; i < count; i++) {
			T = frames[i].basis.get_column(Y).normalized();
			auto prev_N = frames[i - 1].basis.get_column(X);

			// project previous normal onto plane perpendicular to new tangent
			auto proj = prev_N - T * prev_N.dot(T);
			if (Math::is_zero_approx(proj.length())) { // degenerate case, reproject using ref fallback
				proj = ref - T * ref.dot(T);
				if (Math::is_zero_approx(proj.length())) {
					proj = prev_N;
				}
			}

			N = proj.normalized();
			B = T.cross(N).normalized();

			frames[i].basis.set_column(X, N);
			frames[i].basis.set_column(Z, B);
		}
	}
}

int Rope::_frame_at_offset(const LocalVector<Transform3D> &frames, float offset, bool from_end) const {
	// find the nearest transform frame for the given offset
	float frames_per_segment = int(frames.size() / float(_particles.size()));
	float offset_particle = offset * get_particles_per_meter();
	float offset_frame = offset_particle * frames_per_segment;

	// if offset is negative work backwards from the end
	if (from_end)
		offset_frame = (frames.size() - 1) + offset_frame;

	// clamp in range
	offset_frame = Math::clamp(int(offset_frame), 0, int(frames.size() - 1));
	return offset_frame;
}

void Rope::_calculate_frames_for_particles(LocalVector<Transform3D> &frames) const {
	DEV_ASSERT(_particles.size() >= 2);

	Vector3 global_position = get_global_position();
	auto lod = get_rope_lod();
	if (lod < 1)
		lod = 1;

	// got to N-1 because we sample between pairs of particles
	frames.clear();
	for (int i = 0; i < _particles.size(); i++) {
		// 	auto step := get_draw_subdivision_step(camera_position, i);
		const auto particles = _get_control_points_for_particle(i);
		const auto &p0 = particles[0];
		const auto &p1 = particles[1];
		const auto &p2 = particles[2];
		const auto &p3 = particles[3];

		// sample out the catmull spline into points and tangents
		float t = 0.0;
		for (int j = 0; j < lod; j++) {
			auto current_result = _catmull_interpolate(p0, p1, p2, p3, 0.0, t);
			auto position = current_result.first;
			auto tangent = current_result.second;

			// store sampled position and tangent; compute a stable frame after sampling
			// Normal and Binormal are calculated in parallel transport propigation.
			position -= global_position;
			Vector3 zero = Vector3(0, 0, 0);
			frames.push_back(Transform3D(Basis(zero, tangent, zero), position));
			t += 1.0 / float(lod);

			// skip lod frames for the last particle, as it's the terminal frame
			if (i == _particles.size() - 1)
				break;
		}
	}

	// clip the frames by the start and end offsets
	// offset is scaled by rope width so the offset position is independent of rope size
	auto start_offset = get_start_offset() * get_rope_width();
	Basis start_basis = frames[0].basis;
	while (start_offset > 0.0 && frames.size() > 2) {
		// distance to next frame
		Transform3D &frame = frames[0];
		const Transform3D &next = frames[1];
		float dist = frame.origin.distance_to(next.origin);

		// if distance is less than the offset, remove the frame and continue
		if (dist < start_offset) {
			start_offset -= dist;
			frames.remove_at(0);
		} else {
			// move the frame along the line to the next frame
			Vector3 dir = (next.origin - frame.origin).normalized();
			frame.origin += dir * start_offset;
			frame.basis = start_basis;
			break;
		}
	}

	// clip the end offset by walking in reverse.
	auto end_offset = get_end_offset() * get_rope_width();
	Basis end_basis = frames[frames.size() - 1].basis;
	while (end_offset > 0.0 && frames.size() > 2) {
		// distance to next frame
		Transform3D &frame = frames[frames.size() - 1];
		const Transform3D &prev = frames[frames.size() - 2];
		float dist = frame.origin.distance_to(prev.origin);

		// if distance is less than the offset, remove the frame and continue
		if (dist < end_offset) {
			end_offset -= dist;
			frames.remove_at(frames.size() - 1);
		} else {
			// move the frame along the line to the previous frame
			Vector3 dir = (prev.origin - frame.origin).normalized();
			frame.origin += dir * end_offset;
			frame.basis = end_basis;
			break;
		}
	}

	// Need at least two frames to draw a rope
	if (frames.size() >= 2) {
		_compute_parallel_transport(frames);
	}
}

// calculate the transforms for each link so they can be used for both chains and capsule positioning
void Rope::_calculate_links_for_particles(LocalVector<Transform3D> &links) const {
	links.clear();

	// need at least two particles to make a link
	if (_particles.size() >= 2) {
		// N-1 chain links per paricles
		auto count = _particles.size() - 1;
		for (int idx = 0; idx < count; idx++) {
			Vector3 pt = _particles[idx].pos_cur;
			Vector3 pt2 = _particles[idx + 1].pos_cur;
			Vector3 dir = (pt2 - pt);
			float dist = dir.length();
			dir.normalize();

			// xform is at the midpoint between particles
			Transform3D xform(Basis(), pt + dir * dist / 2.0);
			xform.basis.rotate_to_align(Vector3(0, 1, 0), dir);

			// every other frame is rotated along the tangent 90deg
			// so that the links alternate
			if (idx % 2)
				xform = xform.rotated_local(Vector3(0, 1, 0), Math_PI / 2.0);

			// push back the unscaled xform
			links.push_back(xform);
		}
	}
}

#pragma endregion

#pragma region Mesh Rendering
void Rope::_align_attachment_node(const NodePath &path, Transform3D xform, float offset) {
	auto diameter = get_rope_width();
	auto cap_scale = Vector3(diameter, diameter, diameter);
	Node *node = get_node_or_null(path);
	if (node && node->is_class("Node3D")) {
		Node3D *node3d = (Node3D *)node;
		if (node3d->is_visible()) {
			xform.origin += xform.basis.get_column(Y).normalized() * offset;

			node3d->set_global_transform(get_global_transform() * xform.orthonormalized());
			node3d->set_scale(cap_scale);
		}
	}
}

void Rope::_update_aabb() {
	AABB aabb;
	// update mesh and visual instance
	if (_rope_mesh.is_valid()) {
		aabb = _rope_mesh->get_custom_aabb();
	}
	set_custom_aabb(aabb);
}

float Rope::_lod_factor() const {
	// Compute the distance from the main camera to the rope's surface.
	// Build an AABB from particle world positions, expand by rope radius,
	// then measure the shortest distance from the camera to the AABB surface.
	float lod_factor = 1.0f;

	// need at least one particle to compute bounds
	if (_particles.size() == 0)
		return lod_factor;

	// find camera
	Camera3D *cam = nullptr;
	if (Engine::get_singleton()->is_editor_hint()) {
		EditorInterface *interface = EditorInterface::get_singleton();
		SubViewport *viewport = interface->get_editor_viewport_3d(0);
		if (viewport)
			cam = viewport->get_camera_3d();

	} else {
		Viewport *viewport = get_viewport();
		if (viewport)
			cam = viewport->get_camera_3d();
	}
	if (!cam)
		return lod_factor;

	// if camera is orthogonal just return a small constant
	if (cam->get_projection() == Camera3D::PROJECTION_ORTHOGONAL)
		return 1.0f;

	float multiplier = cam->get_camera_projection().get_lod_multiplier();
	Vector3 camera_position = cam->get_global_position();

	// compute aabb from particle positions (world space)
	AABB aabb = get_custom_aabb();
	Vector3 aabb_min = to_global(aabb.position);
	Vector3 aabb_max = to_global(aabb.position + aabb.size);

	// use the rope width as model size basis so that the length of rope doesn't affect LOD
	Vector3 surface_distance = Vector3(0.0, 0.0, 0.0).max(aabb_min - camera_position).max(camera_position - aabb_max);
	float surf_distance = surface_distance.length() * multiplier;
	float model_size = get_rope_width();

	// this 32 is a fudge factor based on taste, i'd like it to be based on something more concrete...
	lod_factor = 1.0 / surf_distance * model_size * get_lod_bias() * 16.0;
	return lod_factor;
}

void Rope::_draw_rope() {
	_update_aabb();

	// Bail early if there are no particles in rope.
	// This can happen if we get a draw call before the Rope is ready
	if (_particles.size() == 0) {
		if (_rope_mesh.is_valid()) {
			_rope_mesh->clear_mesh();
		}
		return;
	}

	// recompute normal, tangents, and binormals
	_compute_particle_normals();

	// recalculate the LOD frames
	_calculate_frames_for_particles(_frames);

	// Need at least two frames to draw a rope
	if (_frames.size() < 2) {
		if (_rope_mesh.is_valid()) {
			_rope_mesh->clear_mesh();
		}
		return;
	}

	auto diameter = get_rope_width();
	const auto radius = diameter * 0.5;

	// tube parameters. set the number of sides based on rope width
	// A 0.1 thickness rope will have 6 sides
	// A 1.0 thickness rope will have 12 sides
	float lod_factor = _lod_factor(); // calucate the lod scaling factor
	int min_sides = MIN(get_rope_sides(), 6); // min LOD sizing to 6 sides
	int sides = CLAMP(get_rope_sides() * lod_factor, min_sides, get_rope_sides());

	// emit geometry
	const int last_frame = _frames.size() - 1;

	// move the link instances if present
	bool has_chain = _instances.size() == _particles.size() - 1 && _instances.size() == _links.size();
	if (has_chain) {
		auto rs = RenderingServer::get_singleton();
		ERR_FAIL_NULL_MSG(rs, "RenderingServer missing");

		// N-1 chain links per particles
		auto count = _links.size();
		Vector3 scale_vec(diameter, diameter, diameter);
		for (int idx = 0; idx < count; idx++) {
			RID instance = _instances[idx];
			Transform3D xform = _links[idx];

			xform.scale_basis(scale_vec);
			rs->instance_set_transform(instance, xform);
		}

		// clear mesh if previous we weren't rendering chain links
		if (_rope_mesh.is_valid()) {
			_rope_mesh->clear_mesh();
		}
	} else if (_rope_mesh.is_valid()) {
		_rope_mesh->set_radius(radius);
		_rope_mesh->set_sides(sides);
		_rope_mesh->set_rope_length(get_rope_length());
		_rope_mesh->set_rope_width(get_rope_width());
		_rope_mesh->set_rope_twist(get_rope_twist());

		// rebuild
		_rope_mesh->_update_mesh_internal(_frames, get_material());
	}

	// align attachments if present
	_align_attachment_node(get_start_attachment(), _frames[0], 0.0);

	// mid attachments
	int count = _get_attachment_count();
	for (int idx = 0; idx < count; idx++) {
		float position = _get_attachment_position(idx);
		if (position != -1) {
			NodePath node = _get_attachment_nodepath(idx);
			Transform3D xform = _get_attachment_transform(idx);

			int index = Math::clamp(int(last_frame * position), 0, last_frame);
			_align_attachment_node(node, _frames[index] * xform, 0.0);
		}
	}

	// end attachment
	Transform3D xform = _frames[last_frame].rotated_local(Vector3(1, 0, 0), Math_PI);
	_align_attachment_node(get_end_attachment(), xform, 0.0);

	update_gizmos();
}

void Rope::_queue_redraw() {
	_dirty = true;
}

bool Rope::_pop_is_dirty() {
	bool is_dirty = _dirty;
	_dirty = false;
	return is_dirty;
}

#pragma endregion

#pragma region Physics

void Rope::_clear_physics_shapes() {
	auto ps = PhysicsServer3D::get_singleton();
	ps->body_clear_shapes(_physics_body);
	for (Particle &p : _particles) {
		if (p.shape.is_valid()) {
			ps->free_rid(p.shape);
			p.shape = RID();
		}
	}
}

void Rope::_rebuild_physics_shapes() {
	auto ps = PhysicsServer3D::get_singleton();
	int particle_count = get_particle_count_for_length();

	Dictionary capsule;
	float radius = get_rope_width() * 0.5f;
	float height = Math::max(get_rope_length() / (particle_count - 1), radius * 2.0f);
	capsule["radius"] = radius;
	capsule["height"] = height + radius * 2;

	// create N-1 capsule colliders
	ps->body_set_enable_continuous_collision_detection(_physics_body, true);
	for (int idx = 0; idx < particle_count - 1; idx++) {
		// physics shape
		RID shape = ps->capsule_shape_create();
		ps->shape_set_data(shape, capsule);
		ps->shape_set_margin(shape, radius / 10.0);
		ps->body_add_shape(_physics_body, shape, Transform3D());
		_particles[idx].shape = shape;
	}

	// clear shape from last particle
	if (particle_count)
		_particles[particle_count - 1].shape = RID();
}

int Rope::get_collision_layer() const { return _collision_layer; }

void Rope::set_collision_layer(int layer) {
	_collision_layer = layer;

	auto ps = PhysicsServer3D::get_singleton();
	ERR_FAIL_NULL(ps);
	ps->body_set_collision_layer(_physics_body, layer);
}

int Rope::get_collision_mask() const { return _collision_mask; }

void Rope::set_collision_mask(int mask) {
	_collision_mask = mask;
	PhysicsServer3D::get_singleton()->body_set_collision_mask(_physics_body, mask);
}

void Rope::_update_physics(float delta, int iterations) {
	for (int i = 0; i < iterations; i++) {
		_apply_forces();
		_apply_constraints();
		_verlet_process(delta);

		_stiff_rope(_stiffness_iterations);

		// now that everything has moved, recalc link positions
		_calculate_links_for_particles(_links);
		_update_collision_shapes();
	}
}

void Rope::_prepare_physics_server() {
	// no-op for now
}

void Rope::_stiff_rope(int iterations) {
	// Calculate the maximum allowed rope length
	const float max_segment_length = _get_average_segment_length();
	const float stiffness = Math::clamp(_stiffness, 0.0f, 2.0f);

#if DEBUG_INITIAL_POS
	static int _once = 10;
	if (_once)
		print_line("Initial _stiff_rope:");
#endif

	for (int j = 0; j < iterations; j++) {
		// First, apply the usual constraints between particles
		for (int i = 0; i < _particles.size() - 1; i++) {
			Particle &p0 = _particles[i];
			Particle &p1 = _particles[i + 1];

			// if both are attached skip
			if (p0.attached && p1.attached) {
				continue;
			}

			const Vector3 segment = p1.pos_cur - p0.pos_cur;
			const Vector3 direction = segment.normalized();
			const float length = segment.length();
			float stretch = (length - max_segment_length);

#if DEBUG_INITIAL_POS
			if (_once) {
				print_line(segment, direction, length, " stretch:", stretch, " adj:", stretch * stiffness);
			}
#endif

			// prevent overshoot
			stretch = Math::clamp(stretch * stiffness, -length, length);

			// If either particle is attached, only move the other one
			if (p0.attached) {
				p1.pos_cur -= direction * stretch;
			} else if (p1.attached) {
				p0.pos_cur += direction * stretch;
			} else {
				const Vector3 half_stretch = direction * 0.5f * stretch;
				p0.pos_cur += half_stretch;
				p1.pos_cur -= half_stretch;
			}
		}
	}

#if DEBUG_INITIAL_POS
	if (_once)
		_once--;
#endif
}

void Rope::_verlet_process(float delta) {
	for (Particle &p : _particles) {
		if (p.attached) {
			continue;
		}

		Vector3 position_current_copy = p.pos_cur;
		p.pos_cur = (2.0 * p.pos_cur) - p.pos_prev + (delta * delta * p.accel);
		p.pos_prev = position_current_copy;
	}
}

void Rope::_apply_forces() {
	// cylinder volume
	float rope_volume = Math_PI * Math::pow(get_rope_width() * 0.5f, 2) * get_rope_length();
	if (rope_volume <= 0.0f)
		rope_volume = 1.0f;

	// NOTE: becuase we don't have mass in verlet, we simulate buoyancy with a ratio.
	// 1.0 == same as water, 0.5 == sink, 1.5 == floaty

	// each point accounts for a portion of the rope volume
	float probe_volume = 0.0f;
	if (_particles.size() > 0) {
		probe_volume = rope_volume / _particles.size();
	}

	// Pre-calculate constants used in the loop
	float rope_width = get_rope_width();
	float rope_radius = rope_width * 0.5f;
	Vector3 gravity_scaled = _gravity * _gravity_scale;
	float pi_r_squared = Math_PI * rope_radius * rope_radius;

	for (Particle &p : _particles) {
		Vector3 total_acceleration = Vector3(0, 0, 0);

		float submerged_ratio = 0.0f;

		// forces act only on unattached
		if (p.attached == false) {
			Vector3 velocity = p.pos_cur - p.pos_prev;

			if (_apply_gravity) {
				total_acceleration += _gravity * _gravity_scale;
			}

			// Because gravity is "unit mass" in verlet, we simulate buoyancy here
			// based on the submerged volume of the rope segment with buoyancy forces as a ratio.
			if (_apply_buoyancy && _liquid_area) {
				Vector3 probe = p.pos_cur;

				// Get wave transform at this position
				Transform3D wave_xform = Transform3D(Basis(), probe);
				wave_xform = _liquid_area->get_liquid_transform(probe);

				// Calculate depths.
				float wave_depth = probe.y - wave_xform.origin.y;

				// calculate the submerged volume for a cylinder section.
				// when depth == 0, rope is half submerged.
				float h = Math::clamp(-wave_depth, 0.0f, get_rope_width());
				float r = get_rope_width() * 0.5f;
				float submerged_area = r * r * Math::acos((r - h) / r) - (r - h) * Math::sqrt(2 * r * h - h * h);
				float submerged_volume = submerged_area * (probe_volume / (Math_PI * r * r));
				submerged_ratio = submerged_volume / probe_volume;

				// If submerged (depth < 0), apply buoyancy force
				if (submerged_ratio > 0.0f) {
					// this is how floaty the rope is...
					float buoyancy_factor = _buoyancy_scale; //liquid_density / _rope_density; ... could also do it this way

					// lerp the wave normal with the up vector based on buoyancy factor
					// this makes the rope float along the surface when partially submerged
					// and float straight up when fully submerged.
					Vector3 wave_normal = wave_xform.basis.get_column(1).normalized();
					wave_normal = wave_normal.slerp(Vector3(0, 1, 0), Math::clamp(1.0f - buoyancy_factor, 0.0f, 1.0f));

					Vector3 wave_force = wave_normal * _gravity * Math::clamp(wave_depth, -1.0f, 0.0f) * buoyancy_factor * submerged_ratio;

					// Apply buoyancy force
					total_acceleration += wave_force;

					// Current and drag forces act on velocity relative to water velocity
					// F_drag = drag_coefficient * (current_velocity - velocity)
					//        = drag_coefficient * current_velocity - drag_coefficient * velocity
					// First term: water current pushes the object
					// Second term: drag slows down motion through water
					Vector3 current_velocity = _liquid_area->get_liquid_velocity();
					Vector3 current_force = current_velocity * CLAMP(_submerged_drag, 0.0, 10.0) * submerged_ratio; // clamped drag range. this is a fudge at the moment because we're not using real mass properties.
					Vector3 drag_force = -velocity * _submerged_drag * submerged_ratio;

					total_acceleration += current_force;
					total_acceleration += drag_force;
				}
			}

			// Wind does not apply when submerged
			if (_apply_wind && _wind_noise != nullptr && submerged_ratio == 0.0f) {
				Vector3 timed_position = p.pos_cur + Vector3(1, 1, 1) * _time;
				float wind_force = _wind_noise->get_noise_3d(timed_position.x, timed_position.y, timed_position.z);
				total_acceleration += _wind_scale * _wind * wind_force;
			}

			// Additional damping
			if (_apply_damping) {
				Vector3 drag = -_damping_factor * velocity.length() * velocity;
				total_acceleration += drag;
			}
		}

		p.accel = total_acceleration;
	}
}

// NOTE: We do *not* use the collider shapes for collision detection of the rope
// instead we cast rays in the direction of travel and model the particles
// as points. This has trade offs, but its significantly faster than a pin_joint + capsule chain.
void Rope::_apply_constraints() {
	// ray cast from the previous position to the current position
	// if we hit something, move the particle to the hit position
	Ref<World3D> w3d = get_world_3d();
	ERR_FAIL_NULL(w3d);

	auto ps = PhysicsServer3D::get_singleton();
	PhysicsDirectSpaceState3D *dss = ps->space_get_direct_state(w3d->get_space());
	ERR_FAIL_NULL(dss);

	// set up the raycast parameters
	if (_collision_mask) {
		// Reuse member ray query to avoid allocation every frame
		_ray_cast->set_collision_mask(_collision_mask);

		float friction = (1.0 - Math::clamp(get_friction(), 0.0f, 1.0f));
		float radius = get_rope_width() * 0.5f;
		for (Particle &p : _particles) {
			if (p.attached)
				continue;

			Vector3 from = p.pos_prev;
			Vector3 to = p.pos_cur;
			Vector3 direction = to - from;

			float length = direction.length();
			if (length < CMP_EPSILON)
				continue;

			_ray_cast->set_from(from);
			_ray_cast->set_to(to);

			Dictionary result = dss->intersect_ray(_ray_cast);
			if (!result.is_empty()) {
				// move the particle to the hit position
				Vector3 position = result["position"];
				Vector3 normal = result["normal"];
				Vector3 contact = position + normal * CMP_EPSILON;

				// // reflect the acceleration and the final position across the normal at the reflection point
				p.accel = p.accel.bounce(normal) * friction;
				p.pos_cur = ((p.pos_cur - contact) * friction).bounce(normal) + contact;

				// TODO: figure out sphere contact point and reflect from there instead.
				// p.pos_cur = reflect_sphere(p.pos_prev, p.pos_cur, normal, contact, radius);
			}
		}
	}
}

// this moves our collision shapes into their new positions
void Rope::_update_collision_shapes() {
	auto ps = PhysicsServer3D::get_singleton();

	// update the shape positions1
	int index = 0;
	int count = ps->body_get_shape_count(_physics_body);
	DEV_ASSERT(count == _links.size());

	for (int idx = 0; idx < count; idx++) {
		// update the shape transform to match the link position
		// NOTE: This is *expensive* to update
		ps->body_set_shape_transform(_physics_body, idx, _links[idx]);
	}
}

#pragma endregion