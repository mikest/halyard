#include "rope.h"
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/physics_direct_space_state3d.hpp>
#include <godot_cpp/classes/physics_ray_query_parameters3d.hpp>
#include <godot_cpp/classes/physics_server3d.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/classes/sphere_shape3d.hpp>
#include <godot_cpp/classes/world3d.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

// Basis indexes
const int X = 0;
const int Y = 1;
const int Z = 2;

Rope::Rope() {
	_generated_mesh.instantiate();
	set_base(_generated_mesh->get_rid());

	_physics_body = PhysicsServer3D::get_singleton()->body_create();
	PhysicsServer3D::get_singleton()->body_set_mode(_physics_body, PhysicsServer3D::BODY_MODE_STATIC);
}

Rope::~Rope() {
	if (_appearance != nullptr)
		_appearance->disconnect("changed", Callable(this, "_on_appearance_changed"));

	_generated_mesh->clear_surfaces();
	_generated_mesh.unref();

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

	EXPORT_PROPERTY(Variant::NODE_PATH, start_anchor, Rope);
	EXPORT_PROPERTY(Variant::NODE_PATH, end_anchor, Rope);
	ClassDB::bind_method(D_METHOD("set_anchors", "anchors"), &Rope::set_anchors);
	ClassDB::bind_method(D_METHOD("get_anchors"), &Rope::get_anchors);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "anchors", PROPERTY_HINT_RESOURCE_TYPE, "RopePositions"), "set_anchors", "get_anchors");

	ClassDB::bind_method(D_METHOD("set_appearance", "appearance"), &Rope::set_appearance);
	ClassDB::bind_method(D_METHOD("get_appearance"), &Rope::get_appearance);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "appearance", PROPERTY_HINT_RESOURCE_TYPE, "RopeAppearance"), "set_appearance", "get_appearance");

	// simulation parameters
	ADD_GROUP("Simulation", "");
	EXPORT_PROPERTY(Variant::BOOL, simulate, Rope);
	EXPORT_PROPERTY(Variant::FLOAT, preprocess_time, Rope);
	EXPORT_PROPERTY(Variant::FLOAT, simulation_rate, Rope);
	EXPORT_PROPERTY_RANGED(Variant::INT, stiffness_iterations, Rope, "1,50,1,hide_slider");
	EXPORT_PROPERTY(Variant::FLOAT, stiffness, Rope);
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
			ERR_FAIL_COND(w3d.is_null());

			RID space = w3d->get_space();
			PhysicsServer3D::get_singleton()->body_set_space(_physics_body, space);

			RID scenario = w3d->get_scenario();
			auto rs = RenderingServer::get_singleton();
			for (auto instance : _instances) {
				rs->instance_set_scenario(instance, scenario);
				rs->instance_set_visible(instance, visible);
			}

		} break;
		case NOTIFICATION_EXIT_WORLD: {
			PhysicsServer3D::get_singleton()->body_set_space(_physics_body, RID());

			auto rs = RenderingServer::get_singleton();
			for (auto instance : _instances) {
				rs->instance_set_scenario(instance, RID());
				rs->instance_attach_skeleton(instance, RID());
				rs->instance_set_visible(instance, visible);
			}

		} break;

		case NOTIFICATION_TRANSFORM_CHANGED: {
			// if (Engine::get_singleton()->is_editor_hint()) {
			// 	// _reset_points_offsets();
			// 	return;
			// }

			// PhysicsServer3D::get_singleton()->body_set_shape_transform(_physics_body, 0, get_global_transform());
		} break;

		case NOTIFICATION_VISIBILITY_CHANGED: {
			// _update_pickable();
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
			// ensure we're always called, even when processing is disabled...
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
	_rebuild_rope();
	_rebuild_mesh();
}

void Rope::_internal_process(double delta) {
	if (_pop_is_dirty()) {
		set_global_transform(Transform3D(Basis(), get_global_position()));
		_rebuild_mesh();
	}
}

void Rope::_internal_physics_process(double delta) {
	_time += delta;
	_simulation_delta += delta;

	auto simulation_step = 1.0 / float(_simulation_rate);
	if (_simulation_delta < simulation_step)
		return;

	// if the particle count has changed, rebuild the rope
	int desired_count = get_particle_count_for_length();
	if (desired_count != _particles.size()) {
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
			_queue_rebuild();
		}
	}

	// reset delta when we simulate
	_simulation_delta = 0.0;
}

void Rope::_clear_instances() {
	auto rs = RenderingServer::get_singleton();
	for (auto &rid : _instances) {
		rs->free_rid(rid);
	}
	_instances.clear();
}

void Rope::_rebuild_instances() {
	auto rs = RenderingServer::get_singleton();
	Ref<World3D> w3d = get_world_3d();
	ERR_FAIL_COND(w3d.is_null());

	if (_appearance != nullptr) {
		auto scenario = get_world_3d()->get_scenario();
		auto mesh = _appearance->get_array_mesh();
		auto material = _appearance->get_material();

		if (mesh != nullptr) {
			// if (material != nullptr && mesh->get_surface_count())
			// 	mesh->surface_set_material(0, material);

			// create one instance for each gap between particles
			// this will be one less than the particle count
			auto count = _particles.size() - 1;

			for (int idx = 0; idx < count; idx++) {
				RID instance = rs->instance_create2(mesh->get_rid(), scenario);

				if (instance.is_valid()) {
					_instances.push_back(instance);
				}
			}
		}
	}
}

void Rope::_rebuild_rope() {
	// free previous shapes
	_clear_physics_shapes();
	_clear_instances();
	_frames.clear();

	auto global_position = get_global_position();
	auto end_location = global_position + _gravity.normalized() * get_rope_length();
	bool end_attached = false;

	Transform3D xform;
	if (_get_anchor_transform(_end_anchor, xform)) {
		end_location = xform.origin;
		end_attached = true;
	}

	const int previous_count = _particles.size();
	int particle_count = get_particle_count_for_length();
	auto segment_length = get_rope_length() / particle_count;

	// grow
	Vector3 direction = (global_position - end_location).normalized();
	Vector3 previous_pos = end_location;

	while (_particles.size() < particle_count) {
		Vector3 position = previous_pos + (direction * segment_length);
		Particle particle;
		particle.pos_prev = position;
		particle.pos_cur = position;
		particle.accel = _gravity * _gravity_scale;
		particle.attached = false;

		// for next particle
		direction = (position - previous_pos).normalized();
		previous_pos = position;

		if (_grow_from > 0)
			_particles.push_back(particle);
		else
			_particles.insert(0, particle);
	}

	// shrink
	if (particle_count > 0) {
		while (_particles.size() > particle_count) {
			if (_grow_from > 0)
				_particles.remove_at(_particles.size() - 1);
			else
				_particles.remove_at(0);
		}
	}

	_clear_physics_shapes();
	_rebuild_physics_shapes();

	Particle &start = _particles[0];
	Particle &end = _particles[_particles.size() - 1];

	start.attached = true;
	end.attached = end_attached;
	end.pos_prev = end_location;
	end.pos_cur = end_location;

	_rebuild_instances();

	_update_anchors();

	// only run preprocess on the first build
	if (previous_count == 0) {
		const float preprocess_delta = 1.0 / _simulation_rate;
		const int preprocess_iterations = Math::max(int(_simulation_rate * _preprocess_time), 1);
		_update_physics(preprocess_delta, preprocess_iterations);
	}

	_compute_particle_normals();

	_queue_rebuild();
}

#pragma endregion

#pragma region Accessors

void Rope::_on_appearance_changed() {
	_rebuild_rope();
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

		_queue_rebuild();
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

#pragma region Anchors & Attachments

void Rope::_update_anchor(NodePath &anchor, float position) {
	int last = _particles.size() - 1;
	int index = Math::clamp(int(last * position), 0, last);

	Transform3D xform;
	if (_get_anchor_transform(anchor, xform)) {
		_particles[index].pos_cur = xform.origin;
		_particles[index].pos_prev = xform.origin;
		_particles[index].attached = true;
	}
}

bool Rope::_get_anchor_transform(const NodePath &path, Transform3D &xform) const {
	Node *node = get_node_or_null(path);
	if (node && node->is_class("Node3D")) {
		Node3D *node3d = (Node3D *)node;
		if (node3d->is_visible()) {
			xform = node3d->get_global_transform();
			return true;
		}
	}

	return false;
}

void Rope::_update_anchors() {
	// for the first anchor, move the whole rope instead of the point
	Transform3D xform;
	if (_get_anchor_transform(_start_anchor, xform)) {
		set_global_position(xform.origin);
	}

	// clear previous position attachments
	for (auto &particle : _particles)
		particle.attached = false;

	// mark the mid anchors
	if (_anchors != nullptr) {
		int count = _anchors->get_position_count();
		for (int idx = 0; idx < count; idx++) {
			auto position = _anchors->get_position_at_index(idx, _rope_length);
			auto node = _anchors->get_node(idx);
			_update_anchor(node, position);
		}
	}

	// attach to the start and end anchors last so that they are always
	// anchored and not stomped on by mid anchors above.
	_update_anchor(_start_anchor, 0.0);
	_update_anchor(_end_anchor, 1.0);
}

#pragma endregion

#pragma region Frame Calculations

float Rope::_get_average_segment_length() const {
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
	auto start_offset = get_start_offset();
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

	// clip the end offset by walking in reverse
	auto end_offset = get_end_offset();
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

void Rope::_rebuild_mesh() {
	_generated_mesh->clear_surfaces();

	// recompute normal, tangents, and binormals
	_compute_particle_normals();

	// recalculate the LOD frames
	_calculate_frames_for_particles(_frames);

	// Need at least two frames to draw a rope
	if (_frames.size() >= 2) {
		// tube parameters. set the number of sides based on rope width
		// A 0.1 thickness rope will have 6 sides
		// A 1.0 thickness rope will have 12 sides
		auto diameter = get_rope_width();
		const auto radius = diameter * 0.5;
		int sides = get_rope_sides();

		// emit geometry
		const int last_frame = _frames.size() - 1;

		// move the link instances if present
		bool has_chain = _instances.size() == _particles.size() - 1 && _instances.size() == _links.size();
		if (has_chain) {
			auto rs = RenderingServer::get_singleton();

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
			// arrays[Mesh::ARRAY_TANGENT] = new_tangents;
			arrays[Mesh::ARRAY_TEX_UV] = uv1s;
			_generated_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLE_STRIP, arrays);
			_generated_mesh->surface_set_material(0, get_material());
		}

		// align attachments if present
		_align_attachment_node(get_start_attachment(), _frames[0], 0.0);

		// mid attachments
		auto attachments = get_attachments();
		if (attachments != nullptr) {
			int count = attachments->get_position_count();
			for (int idx = 0; idx < count; idx++) {
				float position = attachments->get_position_at_index(idx, _rope_length);
				NodePath node = attachments->get_node(idx);

				int index = Math::clamp(int(last_frame * position), 0, last_frame);
				_align_attachment_node(node, _frames[index], 0.0);
			}
		}

		// end attachment
		Transform3D xform = _frames[last_frame].rotated_local(Vector3(1, 0, 0), Math_PI);
		_align_attachment_node(get_end_attachment(), xform, 0.0);
	}

	update_gizmos();
}

#pragma endregion

#pragma region Physics

void Rope::_clear_physics_shapes() {
	PhysicsServer3D::get_singleton()->body_clear_shapes(_physics_body);
	for (Particle &p : _particles) {
		if (p.shape.is_valid()) {
			PhysicsServer3D::get_singleton()->free_rid(p.shape);
			p.shape = RID();
		}
	}
}

void Rope::_rebuild_physics_shapes() {
	int particle_count = get_particle_count_for_length();

	Dictionary capsule;
	float radius = get_rope_width() * 0.5f;
	float height = Math::max(get_rope_length() / (particle_count - 1), radius * 2.0f);
	capsule["radius"] = radius;
	capsule["height"] = height + radius * 2;

	// create N-1 capsule colliders
	for (int idx = 0; idx < particle_count - 1; idx++) {
		// physics shape
		RID shape = PhysicsServer3D::get_singleton()->capsule_shape_create();
		PhysicsServer3D::get_singleton()->shape_set_data(shape, capsule);
		PhysicsServer3D::get_singleton()->body_add_shape(_physics_body, shape, Transform3D());
		_particles[idx].shape = shape;
	}

	// clear shape from last particle
	if (particle_count)
		_particles[particle_count - 1].shape = RID();
}

int Rope::get_collision_layer() const { return _collision_layer; }

void Rope::set_collision_layer(int layer) {
	_collision_layer = layer;
	PhysicsServer3D::get_singleton()->body_set_collision_layer(_physics_body, layer);
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

		_stiff_rope();

		// now that everything has moved, recalc link positions
		_calculate_links_for_particles(_links);
		_update_collision_shapes();
	}
}

void Rope::_prepare_physics_server() {
	// no-op for now
}

void Rope::_stiff_rope() {
	// Calculate the maximum allowed rope length
	const float max_length = get_rope_length() * 1.1f; // Allow 10% stretch, adjust as needed

	for (int j = 0; j < _stiffness_iterations; j++) {
		// First, apply the usual constraints between particles
		for (int i = 0; i < _particles.size() - 1; i++) {
			Particle p0 = _particles[i];
			Particle p1 = _particles[i + 1];

			const Vector3 segment = p1.pos_cur - p0.pos_cur;
			const float segment_length = segment.length();
			const float stretch = segment_length - _get_average_segment_length();
			const float limit = segment_length / 2.0;
			float scalar_stretch = Math::clamp(stretch * _stiffness, -limit, limit);

			Vector3 direction = segment.normalized();

			// If either particle is attached, only move the other one
			if (p0.attached) {
				p1.pos_cur -= direction * scalar_stretch;
			} else if (p1.attached) {
				p0.pos_cur += direction * scalar_stretch;
			} else {
				Vector3 half_stretch = direction * 0.5f * scalar_stretch;
				p0.pos_cur += half_stretch;
				p1.pos_cur -= half_stretch;
			}

			_particles[i] = p0;
			_particles[i + 1] = p1;
		}
	}
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
	Transform3D gx = get_global_transform();
	for (Particle &p : _particles) {
		// p.pos_cur = gx.xform(p.pos_cur);
		// p.pos_prev = gx.xform(p.pos_prev);

		Vector3 total_acceleration = Vector3(0, 0, 0);

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

		p.accel = total_acceleration;
		// p.accel = gx.xform_inv(total_acceleration);
		// p.pos_cur = gx.xform_inv(p.pos_cur);
		// p.pos_prev = gx.xform_inv(p.pos_prev);
	}
}

// NOTE: this doesn't quite work as expected...
Vector3 reflect_sphere(Vector3 start, Vector3 end, Vector3 normal, Vector3 collision, float radius) {
	// Calculate the direction and distance of movement
	Vector3 direction = end - start;
	float distance = direction.length();
	if (distance < CMP_EPSILON) {
		// No movement
		return end;
	}
	direction = direction.normalized();

	// Find the point where the sphere would touch the plane
	// The plane is defined by (collision, normal)
	// Move the sphere center to the point where the sphere's surface touches the plane
	float start_to_plane = (start - collision).dot(normal) - radius;
	float end_to_plane = (end - collision).dot(normal) - radius;

	// If the sphere is moving away from the plane, do nothing
	// if (start_to_plane < 0.0f && end_to_plane < 0.0f) {
	// 	return end;
	// }

	// Find intersection point along the movement vector
	float denom = direction.dot(normal);
	if (Math::is_zero_approx(denom)) {
		// Parallel to the plane, no collision
		return end;
	}

	// t is the fraction along the movement vector where the sphere touches the plane
	float t = ((collision - start).dot(normal) - radius) / denom;
	t = Math::clamp(t, 0.0f, 1.0f);

	// Compute the contact point
	Vector3 contact_point = start + direction * (distance * t);

	// Reflect the remaining movement after collision
	Vector3 remaining = end - contact_point;
	Vector3 reflected = remaining.bounce(normal);

	// New end position after reflection
	Vector3 end_position = contact_point + reflected;

	// Optionally, you could return or output end_position
	// For now, just for demonstration, print it
	return end_position;
}

// NOTE: We do *not* use the collider shapes for collision detection of the rope
// instead we cast rays in the direction of travel and model the particles
// as points. This has trade offs, but its significantly faster than a pin_joint + capsule chain.
void Rope::_apply_constraints() {
	// ray cast from the previous position to the current position
	// if we hit something, move the particle to the hit position
	Ref<World3D> w3d = get_world_3d();
	ERR_FAIL_COND(w3d.is_null());

	PhysicsDirectSpaceState3D *dss = PhysicsServer3D::get_singleton()->space_get_direct_state(w3d->get_space());
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
	// update the shape positions1
	int index = 0;
	int count = PhysicsServer3D::get_singleton()->body_get_shape_count(_physics_body);
	DEV_ASSERT(count == _links.size());

	for (int idx = 0; idx < count; idx++) {
		// update the shape transform to match the link position
		PhysicsServer3D::get_singleton()->body_set_shape_transform(_physics_body, idx, _links[idx]);
	}
}

#pragma endregion