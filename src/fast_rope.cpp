#include "fast_rope.h"
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

FastRope::FastRope() {
	_generated_mesh.instantiate();
	set_base(_generated_mesh->get_rid());
}

FastRope::~FastRope() {
	_generated_mesh->clear_surfaces();
	_generated_mesh.unref();
}

void FastRope::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_baked_mesh"), &FastRope::get_baked_mesh);
	ClassDB::bind_method(D_METHOD("get_current_rope_length"), &FastRope::get_current_rope_length);

#define STR(x) #x
#define EXPORT_PROPERTY(m_type, m_property)                                                              \
	ClassDB::bind_method(D_METHOD(STR(set_##m_property), STR(m_property)), &FastRope::set_##m_property); \
	ClassDB::bind_method(D_METHOD(STR(get_##m_property)), &FastRope::get_##m_property);                  \
	ADD_PROPERTY(PropertyInfo(m_type, #m_property), STR(set_##m_property), STR(get_##m_property))

	EXPORT_PROPERTY(Variant::INT, rope_particles);
	EXPORT_PROPERTY(Variant::FLOAT, rope_width);
	EXPORT_PROPERTY(Variant::FLOAT, rope_length);
	EXPORT_PROPERTY(Variant::INT, rope_sides);
	EXPORT_PROPERTY(Variant::FLOAT, rope_twist);
	EXPORT_PROPERTY(Variant::INT, rope_lod);

	ClassDB::bind_method(D_METHOD("set_material", "material"), &FastRope::set_material);
	ClassDB::bind_method(D_METHOD("get_material"), &FastRope::get_material);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "material", PROPERTY_HINT_RESOURCE_TYPE, "Material"), "set_material", "get_material");

	EXPORT_PROPERTY(Variant::NODE_PATH, start_cap);
	EXPORT_PROPERTY(Variant::NODE_PATH, end_cap);

	// simulation parameters
	ADD_GROUP("Simulation", "");
	EXPORT_PROPERTY(Variant::BOOL, simulate);
	EXPORT_PROPERTY(Variant::FLOAT, simulation_rate);
	EXPORT_PROPERTY(Variant::INT, stiffness_iterations);
	EXPORT_PROPERTY(Variant::FLOAT, stiffness);

	// anchors
	EXPORT_PROPERTY(Variant::NODE_PATH, start_anchor);
	EXPORT_PROPERTY(Variant::NODE_PATH, mid_anchor);
	EXPORT_PROPERTY(Variant::INT, mid_index);
	EXPORT_PROPERTY(Variant::NODE_PATH, end_anchor);

	// forces
	ADD_GROUP("Forces", "");
	EXPORT_PROPERTY(Variant::BOOL, apply_wind);
	EXPORT_PROPERTY(Variant::FLOAT, wind_scale);
	EXPORT_PROPERTY(Variant::VECTOR3, wind);

	ClassDB::bind_method(D_METHOD("set_wind_noise", "wind_noise"), &FastRope::set_wind_noise);
	ClassDB::bind_method(D_METHOD("get_wind_noise"), &FastRope::get_wind_noise);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "wind_noise", PROPERTY_HINT_RESOURCE_TYPE, "FastNoiseLite"), "set_wind_noise", "get_wind_noise");

	EXPORT_PROPERTY(Variant::BOOL, apply_gravity);
	EXPORT_PROPERTY(Variant::VECTOR3, gravity);
	EXPORT_PROPERTY(Variant::FLOAT, gravity_scale);

	EXPORT_PROPERTY(Variant::BOOL, apply_damping);
	EXPORT_PROPERTY(Variant::FLOAT, damping_factor);

	// initial simulation
	ADD_GROUP("Preprocess Simulation", "preprocess_");
	EXPORT_PROPERTY(Variant::INT, preprocess_iterations);
	EXPORT_PROPERTY(Variant::FLOAT, preprocess_delta);

#undef EXPORT_PROPERTY
#undef STR
}

void FastRope::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_READY: {
			// ensure we're always called, even when processing is disabled...
			set_process_internal(true);
			_rebuild_mesh();
		} break;

		case NOTIFICATION_INTERNAL_PROCESS: {
			if (_pop_is_dirty()) {
				set_global_transform(Transform3D(Basis(), get_global_position()));
				_rebuild_mesh();
			}
		} break;
	}
}

void FastRope::_ready(void) {
	_create_rope();
}

void FastRope::_update_anchor(NodePath &anchor, int index) {
	index = Math::clamp(index, 0, int(_particles.size() - 1));

	Transform3D xform;
	if (_get_anchor_transform(anchor, xform)) {
		_particles[index].pos_cur = xform.origin;
		_particles[index].attached = true;
	} else
		_particles[index].attached = false;
}

void FastRope::_physics_process(double delta) {
	_time += delta;
	_simulation_delta += delta;

	auto simulation_step = 1.0 / float(simulation_rate);
	if (_simulation_delta < simulation_step)
		return;

	if (_particles.size() >= 2) {
		// unattach all the particles
		for (Particle &p : _particles)
			p.attached = false;

		// set the anchor point for the beginning
		_update_anchor(start_anchor, 0);
		_update_anchor(mid_anchor, mid_index);
		_update_anchor(end_anchor, _particles.size() - 1);

		// run simulation
		if (simulate) {
			_apply_forces();
			_verlet_process(_simulation_delta);
			_apply_constraints();

			_queue_rebuild();
		}
	}

	// reset delta when we simulate
	_simulation_delta = 0.0;
}

#pragma mark -

void FastRope::set_material(const Ref<Material> &p_material) {
	material = p_material;
	if (_generated_mesh.is_valid() && _generated_mesh->get_surface_count() > 0) {
		_generated_mesh->surface_set_material(0, material);
	}
}

Ref<Material> FastRope::get_material() const {
	return material;
}

void FastRope::set_wind_noise(const Ref<FastNoiseLite> &p_noise) {
	wind_noise = p_noise;
}

Ref<FastNoiseLite> FastRope::get_wind_noise() const {
	return wind_noise;
}

#pragma mark -

void FastRope::_create_rope() {
	auto global_position = get_global_position();
	auto end_location = global_position + Vector3(0, -1, 0) * rope_length;
	bool end_attached = false;

	Transform3D xform;
	if (_get_anchor_transform(end_anchor, xform)) {
		end_location = xform.origin;
		end_attached = true;
	}

	auto acceleration = gravity * gravity_scale;
	auto segment = _get_average_segment_length();

	_build_particles(
			end_location,
			global_position,
			acceleration,
			rope_particles,
			segment);

	Particle &start = _particles[0];
	Particle &end = _particles[_particles.size() - 1];

	start.attached = true;
	end.attached = end_attached; //_attach_end != null;
	end.pos_prev = end_location;
	end.pos_cur = end_location;

	_update_anchor(start_anchor, 0);
	_update_anchor(mid_anchor, mid_index);
	_update_anchor(end_anchor, _particles.size() - 1);

	for (int i = 0; i < preprocess_iterations; i++) {
		_verlet_process(preprocess_delta);
		_apply_constraints();
	}

	_compute_particle_normals();
	_queue_rebuild();
}

void FastRope::_build_particles(const Vector3 &end_location, const Vector3 &global_position, const Vector3 &initial_accel, int particle_count, float segment_length) {
	Vector3 direction = (end_location - global_position).normalized();

	_particles.clear();
	for (int i = 0; i < particle_count; i++) {
		Particle particle;
		Vector3 position = global_position + direction * segment_length * i;
		particle.pos_prev = position;
		particle.pos_cur = position;
		particle.accel = initial_accel;
		particle.attached = false;
		_particles.push_back(particle);
	}
}

void FastRope::set_particle_count(uint64_t p_count) {
	uint64_t cur_size = _particles.size();
	if (cur_size != p_count) {
		if (p_count > cur_size) {
			uint64_t add_count = p_count - cur_size;
			for (uint64_t i = 0; i < add_count; i++) {
				_particles.push_back(Particle());
			}
		} else {
			_particles.resize(p_count);
		}
		_queue_rebuild();
	}
}

uint64_t FastRope::get_particle_count() const {
	return _particles.size();
}

Ref<ArrayMesh> FastRope::get_baked_mesh() const {
	return _generated_mesh->duplicate();
}

#pragma mark -

const int X = 0;
const int Y = 1;
const int Z = 2;

Transform3D _align_up(const Transform3D &in_xform, const Vector3 &normal) {
	Transform3D xform = in_xform;

	xform.basis.set_column(Y, normal);
	xform.basis.set_column(X, -in_xform.basis.get_column(Z).cross(normal));
	xform.basis = xform.basis.orthonormalized();
	return xform;
}

void FastRope::_compute_particle_normals() {
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
void FastRope::_compute_parallel_transport(LocalVector<Transform3D> &frames) {
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

void FastRope::_emit_tube(LocalVector<Transform3D> &frames, int sides, float radius, PackedVector3Array &V, PackedVector3Array &N, PackedVector2Array &UV1) {
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
	const float repeats = rope_length / rope_width * rope_twist;

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

void FastRope::_emit_endcap(bool front, const Transform3D &frame, int sides, float radius, PackedVector3Array &mesh_v, PackedVector3Array &mesh_n, PackedVector2Array &mesh_uv1) {
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

bool FastRope::_get_anchor_transform(const NodePath &path, Transform3D &xform) const {
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

void FastRope::_align_cap_node(const NodePath &path, Transform3D xform) {
	auto cap_scale = Vector3(rope_width, rope_width, rope_width);
	Node *node = get_node_or_null(path);
	if (node && node->is_class("Node3D")) {
		Node3D *node3d = (Node3D *)node;
		if (node3d->is_visible()) {
			node3d->set_global_transform(get_global_transform() * xform.orthonormalized());
			node3d->set_scale(cap_scale);
		}
	}
}

void FastRope::_rebuild_mesh() {
	_generated_mesh->clear_surfaces();

	// recompute normal, tangents, and binormals
	_compute_particle_normals();

	// build the tranform frames from the catmull spline
	Vector3 global_position = get_global_position();
	LocalVector<Transform3D> frames;
	for (int i = 0; i < _particles.size() - 1; i++) {
		const auto particles = _get_simulation_particles(i);
		const auto &p0 = particles[0];
		const auto &p1 = particles[1];
		const auto &p2 = particles[2];
		const auto &p3 = particles[3];

		// 	auto step := get_draw_subdivision_step(camera_position, i);
		float step = 1.0;
		float t = 0.0;

		// adjustable lod
		if (rope_lod > 0)
			step = 1.0 / float(rope_lod);

		// sample out the catmull spline into points and tangents
		while (t <= 1.0) {
			auto current_result = _catmull_interpolate(p0, p1, p2, p3, 0.0, t);
			auto position = current_result.first;
			auto tangent = current_result.second;

			// store sampled position and tangent; compute a stable frame after sampling
			position -= global_position;
			// Transform3D for each point.
			// Tangent is Y
			// Normal is X, calculated in parallel transport
			// Binormal is Z, calculated in parallel transport
			Vector3 zero = Vector3(0, 0, 0);
			frames.push_back(Transform3D(Basis(zero, tangent, zero), position));
			t += step;
		}
	}

	// Need at least two frames to draw a rope
	if (frames.size() >= 2) {
		_compute_parallel_transport(frames);

		// tube parameters. set the number of sides based on rope width
		// A 0.1 thickness rope will have 6 sides
		// A 1.0 thickness rope will have 12 sides
		// float ratio = Math::ease(rope_width, .2);
		float ratio = 0.5;
		int sides = Math::clamp(int(Math::lerp(3, 12, ratio)), 3, 12);
		auto radius = rope_width * 0.5;

		// override
		if (rope_sides >= 0) {
			sides = rope_sides;
		}

		PackedVector3Array verts;
		PackedVector3Array norms;
		PackedVector2Array uv1s;

		_emit_endcap(true, frames[0], sides, radius, verts, norms, uv1s);
		_align_cap_node(start_cap, frames[0]);

		_emit_tube(frames, sides, radius, verts, norms, uv1s);

		auto idx = frames.size() - 1;
		_emit_endcap(false, frames[idx], sides, radius, verts, norms, uv1s);
		_align_cap_node(end_cap, frames[idx].rotated_local(Vector3(1, 0, 0), Math_PI));

		// generate the catmull interpolation for the segments.
		Array arrays;
		arrays.resize(Mesh::ARRAY_MAX);
		arrays.fill(Variant());
		arrays[Mesh::ARRAY_VERTEX] = verts;
		arrays[Mesh::ARRAY_NORMAL] = norms;
		// arrays[Mesh::ARRAY_TANGENT] = new_tangents;
		arrays[Mesh::ARRAY_TEX_UV] = uv1s;
		_generated_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLE_STRIP, arrays);
		_generated_mesh->surface_set_material(0, material);
	}
}

#pragma mark -

PackedVector3Array FastRope::_get_simulation_particles(int index) {
	auto segment_length = _get_average_segment_length();

	PackedVector3Array p;
	p.resize(4);

	if (index == 0)
		p[0] = _particles[index].pos_cur - (_particles[index].T * segment_length);
	else
		p[0] = _particles[index - 1].pos_cur;

	p[1] = _particles[index].pos_cur;
	p[2] = _particles[index + 1].pos_cur;

	if (index == rope_particles - 2)
		p[3] = _particles[index + 1].pos_cur + (_particles[index + 1].T * segment_length);
	else
		p[3] = _particles[index + 2].pos_cur;

	return p;
}

Pair<Vector3, Vector3> FastRope::_catmull_interpolate(const Vector3 &p0, const Vector3 &p1, const Vector3 &p2, const Vector3 &p3, float tension, float t) {
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

float FastRope::get_current_rope_length() const {
	float length = 0.0;
	for (int i = 0; i < _particles.size() - 1; i++)
		length += (_particles[i + 1].pos_cur - _particles[i].pos_cur).length();
	return length;
}

float FastRope::_get_average_segment_length() const {
	return rope_length / float(_particles.size() - 1);
}

void FastRope::_stiff_rope() {
	for (int j = 0; j < stiffness_iterations; j++) {
		for (int i = 0; i < _particles.size() - 1; i++) {
			Particle p0 = _particles[i];
			Particle p1 = _particles[i + 1];

			Vector3 segment = p1.pos_cur - p0.pos_cur;
			float stretch = segment.length() - _get_average_segment_length();
			Vector3 direction = segment.normalized();

			if (p0.attached) {
				p1.pos_cur -= direction * stretch * stiffness;
			} else if (p1.attached) {
				p0.pos_cur += direction * stretch * stiffness;
			} else {
				Vector3 half_stretch = direction * stretch * 0.5 * stiffness;
				p0.pos_cur += half_stretch;
				p1.pos_cur -= half_stretch;
			}

			_particles[i] = p0;
			_particles[i + 1] = p1;
		}
	}
}

void FastRope::_verlet_process(float delta) {
	for (Particle &p : _particles) {
		if (p.attached) {
			continue;
		}

		Vector3 position_current_copy = p.pos_cur;
		p.pos_cur = (2.0 * p.pos_cur) - p.pos_prev + (delta * delta * p.accel);
		p.pos_prev = position_current_copy;
	}
}

void FastRope::_apply_forces() {
	Transform3D gx = get_global_transform();
	for (Particle &p : _particles) {
		// p.pos_cur = gx.xform(p.pos_cur);
		// p.pos_prev = gx.xform(p.pos_prev);

		Vector3 total_acceleration = Vector3(0, 0, 0);

		if (apply_gravity) {
			total_acceleration += gravity * gravity_scale;
		}

		if (apply_wind && wind_noise != nullptr) {
			Vector3 timed_position = p.pos_cur + Vector3(1, 1, 1) * _time;
			float wind_force = wind_noise->get_noise_3d(timed_position.x, timed_position.y, timed_position.z);
			total_acceleration += wind_scale * wind * wind_force;
		}
		if (apply_damping) {
			Vector3 velocity = p.pos_cur - p.pos_prev;
			Vector3 drag = -damping_factor * velocity.length() * velocity;
			total_acceleration += drag;
		}

		p.accel = total_acceleration;
		// p.accel = gx.xform_inv(total_acceleration);
		// p.pos_cur = gx.xform_inv(p.pos_cur);
		// p.pos_prev = gx.xform_inv(p.pos_prev);
	}
}

void FastRope::_apply_constraints() {
	_stiff_rope();
}
