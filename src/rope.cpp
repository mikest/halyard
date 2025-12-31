#include "rope.h"
#include "rope_anchors_base.h"
#include "rope_appearance.h"
#include "rope_attachments_base.h"
#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/editor_interface.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/physics_direct_space_state3d.hpp>
#include <godot_cpp/classes/physics_ray_query_parameters3d.hpp>
#include <godot_cpp/classes/physics_server3d.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/classes/sphere_shape3d.hpp>
#include <godot_cpp/classes/sub_viewport.hpp>
#include <godot_cpp/classes/viewport.hpp>
#include <godot_cpp/classes/world3d.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

// dump initial conditions for debugging stretch grow directions.
#define DEBUG_INITIAL_POS false

// Basis indexes
const int X = 0;
const int Y = 1;
const int Z = 2;

Rope::Rope() {
	_generated_mesh.instantiate();
	set_base(_generated_mesh->get_rid());

	auto ps = PhysicsServer3D::get_singleton();
	_physics_body = ps->body_create();
	ps->body_set_mode(_physics_body, PhysicsServer3D::BODY_MODE_STATIC);
}

Rope::Rope(const Rope &other) {
}

Rope::~Rope() {
	if (_appearance != nullptr)
		_appearance->disconnect("changed", Callable(this, "_on_appearance_changed"));

	if (_generated_mesh.is_valid()) {
		_generated_mesh->clear_surfaces();
		_generated_mesh.unref();
	}

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
	EXPORT_PROPERTY(Variant::BOOL, apply_wind, Rope);
	EXPORT_PROPERTY(Variant::FLOAT, wind_scale, Rope);
	EXPORT_PROPERTY(Variant::VECTOR3, wind, Rope);

	ClassDB::bind_method(D_METHOD("set_wind_noise", "wind_noise"), &Rope::set_wind_noise);
	ClassDB::bind_method(D_METHOD("get_wind_noise"), &Rope::get_wind_noise);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "wind_noise", PROPERTY_HINT_RESOURCE_TYPE, "FastNoiseLite"), "set_wind_noise", "get_wind_noise");

	EXPORT_PROPERTY(Variant::BOOL, apply_gravity, Rope);
	EXPORT_PROPERTY(Variant::VECTOR3, gravity, Rope);
	EXPORT_PROPERTY(Variant::FLOAT, gravity_scale, Rope);

	EXPORT_PROPERTY(Variant::BOOL, apply_damping, Rope);
	EXPORT_PROPERTY(Variant::FLOAT, damping_factor, Rope);
}

#pragma region Lifecycle

void Rope::_notification(int p_what) {
	double delta = Engine::get_singleton()->is_in_physics_frame() ? get_physics_process_delta_time() : get_process_delta_time();
	bool visible = is_visible_in_tree();

	switch (p_what) {
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

		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS:
			_internal_physics_process(delta);
			break;

		case NOTIFICATION_INTERNAL_PROCESS:
			_internal_process(delta);
			break;

		case NOTIFICATION_EXIT_TREE: {
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
	int insert_idx = 0;
	while (_particles.size() < particle_count) {
		Particle particle;
		// NOTE: this flag generates a small initial offset to the calculated previous position
		// so the simulation isn't "perfectly aligned" which can cause problems growing the rope
		// in a direction perfectly aligned with the gravity vector.
		//
		// If the row grows too slowly "stacking" can still occur as the stiffness will stablize it.
		Vector3 jitter_prev = (_jitter_initial_position ? _small_offset(segment_length / 10.0) : Vector3());
		Vector3 jitter_cur = (_jitter_initial_position ? _small_offset(segment_length / 10.0) : Vector3());
		particle.pos_prev = current_pos + jitter_prev;
		particle.pos_cur = current_pos + jitter_cur;
		particle.attached = false;

		// for next particle
		auto jitter_dir = (direction + (_jitter_initial_position ? _small_offset(0.1) : Vector3())).normalized();
		current_pos = current_pos + (direction * segment_length);

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
	return _generated_mesh->duplicate();
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

void Rope::_emit_tube(LocalVector<Transform3D> &frames, int start, int end, int sides, float radius, PackedVector3Array &V, PackedVector3Array &N, PackedVector2Array &UV1) {
	// build cumulative length along the sampled positions so we can map V smoothly.
	// rope can be stretchy so we can't just use rope_length here
	LocalVector<float> cum_lengths;
	cum_lengths.push_back(0.0);
	for (int k = 1; k < frames.size(); k++)
		cum_lengths.push_back(cum_lengths[k - 1] + frames[k - 1].origin.distance_to(frames[k].origin));

	float total_length = cum_lengths[cum_lengths.size() - 1];
	if (total_length <= 0.0)
		total_length = 1.0;

	// number of V repeats along the rope is based on rope width and twist factor
	const float repeats = get_rope_length() / get_rope_width() * get_rope_twist();

	// NOTE: run to the second to the last frame as we emit 2 frames at a time.
	for (int i = 0; i < frames.size() - 1; i++) {
		const auto &pos = frames[i].origin;
		const auto &next_pos = frames[i + 1].origin;

		const auto &norm = frames[i].basis.get_column(X);
		const auto &binorm = frames[i].basis.get_column(Z);
		const auto &next_norm = frames[i + 1].basis.get_column(X);
		const auto &next_binorm = frames[i + 1].basis.get_column(Z);

		const auto v = (cum_lengths[i] / total_length) * repeats;
		const auto next_v = (cum_lengths[i + 1] / total_length) * repeats;

		// loop one extra to close the seam (repeat first vertex)
		// for the first and the last row.
		for (int j = sides; j >= 0; j--) {
			const auto wrap_j = j % sides;
			const auto angle = Math_TAU * float(wrap_j) / float(sides);
			const auto ca = cos(angle);
			const auto sa = sin(angle);

			const auto offset = (binorm * ca + norm * sa) * radius;
			const auto next_offset = (next_binorm * ca + next_norm * sa) * radius;

			const auto normal = offset.normalized();
			const auto next_normal = next_offset.normalized();

			// U goes 0..1 around the tube; use j so the final seam vertex reaches 1.0;
			const auto u = float(j) / float(sides);

			V.push_back(pos + offset);
			N.push_back(normal);
			UV1.push_back(Vector2(u, v));

			V.push_back(next_pos + next_offset);
			N.push_back(next_normal);
			UV1.push_back(Vector2(u, next_v));
		}
	}
}

void Rope::_emit_endcap(bool front, const Transform3D &frame, int sides, float radius, PackedVector3Array &mesh_v, PackedVector3Array &mesh_n, PackedVector2Array &mesh_uv1) {
	Vector3 center = frame.origin;
	Vector3 T = frame.basis.get_column(Y);
	Vector3 N = frame.basis.get_column(X);
	Vector3 B = frame.basis.get_column(Z);

	// UV to be aligned radially around the rope edge as a function of the side count
	float u_width = 1.0 / sides;
	Vector3 center_normal = T.normalized() * (front ? -1.0 : 1.0);

	auto emit = [&](int j) {
		const auto wrap_j = j % sides;
		const auto angle = Math_TAU * float(wrap_j) / float(sides);
		const auto ca = cos(angle);
		const auto sa = sin(angle);
		const auto a = center + (B * ca + N * sa) * radius;
		const auto uv_a = Vector2(wrap_j * u_width, 0);

		const auto center_uv = Vector2(j * u_width, 0);

		mesh_v.push_back(a);
		mesh_n.push_back(center_normal);
		mesh_uv1.push_back(uv_a);

		mesh_v.push_back(center);
		mesh_n.push_back(center_normal);
		mesh_uv1.push_back(center_uv);
	};

	// emit triangles for end cap in either CCW or CW order
	if (front) {
		for (int j = 0; j < sides + 1; j++)
			emit(j);
	} else {
		for (int j = sides; j >= 0; j--) {
			emit(j);
		}
	}
}

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
	// need at least one particle to compute bounds
	if (_particles.size() == 0) {
		aabb = AABB();
		return;
	}

	// compute aabb from particle positions (local space)
	Vector3 pos_cur = to_local(_particles[0].pos_cur);
	Vector3 aabb_min = pos_cur;
	Vector3 aabb_max = pos_cur;
	for (int i = 1; i < _particles.size(); i++) {
		pos_cur = to_local(_particles[i].pos_cur);
		aabb_min = aabb_min.min(pos_cur);
		aabb_max = aabb_max.max(pos_cur);
	}

	// expand by rope radius
	float r = get_rope_width() * 0.5f;
	aabb_min -= Vector3(r, r, r);
	aabb_max += Vector3(r, r, r);

	aabb.position = aabb_min;
	aabb.size = aabb_max - aabb_min;

	// update mesh and visual instance
	_generated_mesh->set_custom_aabb(aabb);
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
	_generated_mesh->clear_surfaces();
	_update_aabb();

	// nothing to draw
	if (_particles.size() == 0)
		return;

	// Bail early if there are no particles in rope.
	// This can happen if we get a draw call before the Rope is ready
	if (_particles.size() == 0) {
		WARN_PRINT("Skipping drawing, particle count is zero.");
		return;
	}

	// recompute normal, tangents, and binormals
	_compute_particle_normals();

	// recalculate the LOD frames
	_calculate_frames_for_particles(_frames);

	// Need at least two frames to draw a rope
	if (_frames.size() >= 2) {
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
			for (int idx = 0; idx < count; idx++) {
				RID instance = _instances[idx];
				Transform3D xform = _links[idx];

				xform.scale_basis(Vector3(diameter, diameter, diameter));
				rs->instance_set_transform(instance, xform);
			}

			// render gen mesh
		} else {
			PackedVector3Array verts, norms;
			PackedVector2Array uv1s;

			_emit_endcap(true, _frames[0], sides, radius, verts, norms, uv1s);
			_emit_tube(_frames, 0, last_frame, sides, radius, verts, norms, uv1s);
			_emit_endcap(false, _frames[last_frame], sides, radius, verts, norms, uv1s);

			// generate the catmull interpolation for the segments.
			Array arrays;
			arrays.resize(Mesh::ARRAY_MAX);
			arrays.fill(Variant());
			arrays[Mesh::ARRAY_VERTEX] = verts;
			arrays[Mesh::ARRAY_NORMAL] = norms;
			arrays[Mesh::ARRAY_TEX_UV] = uv1s;
			_generated_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLE_STRIP, arrays);
			_generated_mesh->surface_set_material(0, get_material());
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
	}

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
	for (int idx = 0; idx < particle_count - 1; idx++) {
		// physics shape
		RID shape = ps->capsule_shape_create();
		ps->shape_set_data(shape, capsule);
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
	for (Particle &p : _particles) {
		Vector3 total_acceleration = Vector3(0, 0, 0);

		// forces act only on unattached
		if (p.attached == false) {
			if (_apply_gravity) {
				total_acceleration += _gravity * _gravity_scale;
			}

			if (_apply_wind && _wind_noise != nullptr) {
				Vector3 timed_position = p.pos_cur + Vector3(1, 1, 1) * _time;
				float wind_force = _wind_noise->get_noise_3d(timed_position.x, timed_position.y, timed_position.z);
				total_acceleration += _wind_scale * _wind * wind_force;
			}

			if (_apply_damping) {
				Vector3 velocity = p.pos_cur - p.pos_prev;
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
		Ref<PhysicsRayQueryParameters3D> rq;
		rq.instantiate();

		TypedArray<RID> exclude;
		exclude.append(_physics_body);
		rq->set_exclude(exclude);
		rq->set_collision_mask(_collision_mask);
		rq->set_collide_with_areas(false);
		rq->set_collide_with_bodies(true);

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

			rq->set_from(from);
			rq->set_to(to);

			Dictionary result = dss->intersect_ray(rq);
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
		ps->body_set_shape_transform(_physics_body, idx, _links[idx]);
	}
}

#pragma endregion