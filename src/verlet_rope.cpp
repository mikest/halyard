#include "verlet_rope.h"
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

VerletRope::VerletRope() {
	_generated_mesh.instantiate();
	set_base(_generated_mesh->get_rid());
}

VerletRope::~VerletRope() {
	_generated_mesh->clear_surfaces();
	_generated_mesh.unref();
}

void VerletRope::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_baked_mesh"), &VerletRope::get_baked_mesh);
	// ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "baked_mesh", PROPERTY_HINT_RESOURCE_TYPE, "", PROPERTY_USAGE_READ_ONLY, "ArrayMesh"), "", "get_baked_mesh");

	ClassDB::bind_method(D_METHOD("set_material", "material"), &VerletRope::set_material);
	ClassDB::bind_method(D_METHOD("get_material"), &VerletRope::get_material);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "material", PROPERTY_HINT_RESOURCE_TYPE, "Material"), "set_material", "get_material");
}

void VerletRope::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_READY: {
			// ensure we're always called, even when processing is disabled...
			set_process_internal(true);
			_rebuild_mesh();
		} break;

		case NOTIFICATION_INTERNAL_PROCESS: {
			if (_pop_is_dirty()) {
				_rebuild_mesh();
			}
		} break;
	}
}

void VerletRope::_ready(void) {
	_create_rope();
}

void VerletRope::_process(double delta) {
}

void VerletRope::_physics_process(double delta) {
	_time += delta;
	_simulation_delta += delta;

	auto simulation_step = 1.0 / float(simulation_rate);
	if (_simulation_delta < simulation_step)
		return;

	if (attach_start && _particles.size()) {
		_particles[0].pos_cur = get_global_position();
	}

	if (simulate) {
		_apply_forces();
		_verlet_process(_simulation_delta);
		_verlet_process(0.16);
		_apply_constraints();

		_queue_rebuild();
	}

	// reset delta when we simulate
	_simulation_delta = 0.0;
}

#pragma mark -

void VerletRope::set_material(const Ref<Material> &p_material) {
	material = p_material;
	if (_generated_mesh.is_valid() && _generated_mesh->get_surface_count() > 0) {
		_generated_mesh->surface_set_material(0, material);
	}
}

Ref<Material> VerletRope::get_material() const {
	return material;
}

#pragma mark -

void VerletRope::_create_rope() {
	auto global_position = get_global_position();
	auto end_location = global_position + Vector3(0, -1, 0) * rope_length;

	// if (_attach_end != null):
	// 	end_location = _attach_end.global_position;
	// elif start_simulation_from_start_point:
	// end_location = global_position;

	auto acceleration = gravity * gravity_scale;
	auto segment = _get_average_segment_length();

	_build_particles(
			end_location,
			global_position,
			acceleration,
			simulation_particles,
			segment);

	Particle &start = _particles[0];
	Particle &end = _particles[_particles.size() - 1];

	start.attached = attach_start;
	end.attached = false; //_attach_end != null;
	end.pos_prev = end_location;
	end.pos_cur = end_location;

	for (int i = 0; i < preprocess_iterations; i++) {
		_verlet_process(preprocess_delta);
		_apply_constraints();
	}

	_queue_rebuild();
}

void VerletRope::_build_particles(const Vector3 &end_location, const Vector3 &global_position, const Vector3 &initial_accel, int particle_count, float segment_length) {
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

void VerletRope::set_particle_count(uint64_t p_count) {
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

uint64_t VerletRope::get_particle_count() const {
	return _particles.size();
}

Ref<ArrayMesh> VerletRope::get_baked_mesh() const {
	return _generated_mesh->duplicate();
}

#pragma mark -

const int X = 0;
const int Y = 1;
const int Z = 2;

Transform3D _align_up(const Transform3D &in_xform, const Vector3 &normal) {
	Transform3D xform = in_xform;
	xform.basis[Y] = normal;
	xform.basis[X] = -xform.basis[Z].cross(normal);
	xform.basis = xform.basis.orthonormalized();
	return xform;
}

// compute a stable normal/binormal frame using parallel transport to reduce twisting
// ref: https://en.wikipedia.org/wiki/Parallel_transport
void VerletRope::_compute_parallel_transport(LocalVector<Transform3D> &frames) {
	auto count = frames.size();
	if (count > 0) {
		Vector3 ref; // reference we'll use to align the rest of the frames
		Vector3 T;
		Vector3 N;
		Vector3 B;

		if (false) {
			// initial tangent
			T = frames[0].basis[Y].normalized();

			// choose an arbitrary reference not parallel to t0
			ref = Vector3(0, 1, 0); // Vector3.UP
			if (abs(T.dot(ref)) > 0.999)
				ref = Vector3(0, 0, -1); // Vector3.MODEL_FRONT

			// project ref onto plane perpendicular to t0
			N = (ref - T * ref.dot(T));
			if (N.length() <= 0.0)
				N = Vector3(1, 0, 0);
			N = N.normalized();

			B = T.cross(N).normalized();
		} else {
			// use the basis for our rope start aligned to the first tangent
			// as the reference frame. this keeps the normals and U coordinate
			// from swinging wildly when the curvature changes orientation

			Transform3D global_tansform = get_global_transform();
			auto gb = _align_up(global_tansform, frames[0].basis[Y].normalized()).basis;
			ref = -gb[Y];
			N = gb[X];
			B = -gb[Z];
		}

		frames[0].basis[X] = N;
		frames[0].basis[Z] = B;

		// parallel transport subsequent normals
		for (int i = 1; i < count; i++) {
			T = frames[i].basis[Y].normalized();
			auto prev_N = frames[i - 1].basis[X];

			// project previous normal onto plane perpendicular to new tangent
			auto proj = prev_N - T * prev_N.dot(T);
			if (Math::is_zero_approx(proj.length())) { // degenerate case, reproject using ref fallback
				proj = (ref - T * ref.dot(T));
				if (Math::is_zero_approx(proj.length())) {
					proj = prev_N;
				}
			}

			N = proj.normalized();
			B = T.cross(N).normalized();

			frames[i].basis[X] = N;
			frames[i].basis[Z] = B;
		}
	}
}

void VerletRope::_emit_tube(LocalVector<Transform3D> &frames, int sides, float radius, PackedVector3Array &V, PackedVector3Array &N, PackedVector2Array &UV1) {
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
	float repeats = rope_length / rope_width * rope_twist;

	for (int i = 0; i < frames.size() - 1; i++) {
		auto pos = frames[i].origin;
		auto next_pos = frames[i + 1].origin;

		auto norm = frames[i].basis[X];
		auto binorm = frames[i].basis[Z];
		auto next_norm = frames[i + 1].basis[X];
		auto next_binorm = frames[i + 1].basis[Z];

		auto v = (cum_lengths[i] / total_length) * repeats;
		auto next_v = (cum_lengths[i + 1] / total_length) * repeats;

		// loop one extra to close the seam (repeat first vertex)
		for (int j = 0; j < sides + 1; j++) {
			auto wrap_j = j % sides;
			auto angle = Math_TAU * float(wrap_j) / float(sides);
			auto ca = cos(angle);
			auto sa = sin(angle);

			auto offset = (binorm * ca + norm * sa) * radius;
			auto next_offset = (next_binorm * ca + next_norm * sa) * radius;

			auto normal = offset.normalized();
			auto next_normal = next_offset.normalized();

			// U goes 0..1 around the tube; use j so the final seam vertex reaches 1.0;
			auto u = float(j) / float(sides);

			// reverse winding: emit next vertex then current vertex per pair
			V.push_back(next_pos + next_offset);
			N.push_back(next_normal);
			UV1.push_back(Vector2(u, next_v));

			V.push_back(pos + offset);
			N.push_back(normal);
			UV1.push_back(Vector2(u, v));

			// _add_vertex_with_normal_and_uv(next_pos + next_offset, next_normal, Vector2(u, next_v))
			// _add_vertex_with_normal_and_uv(pos + offset, normal, Vector2(u, v))
		}
	}
}

void _emit_endcap(bool front, const Transform3D &frame, int sides, float radius, PackedVector3Array &V, PackedVector3Array &N, PackedVector2Array &UV1) {
}

void VerletRope::_rebuild_mesh() {
	_generated_mesh->clear_surfaces();

	// build the tranform frames from the catmull spline
	Vector3 global_position = get_global_position();
	LocalVector<Transform3D> frames;
	for (int i = 0; i < _particles.size() - 1; i++) {
		auto particles = _get_simulation_particles(i);
		auto p0 = particles[0];
		auto p1 = particles[1];
		auto p2 = particles[2];
		auto p3 = particles[3];

		// 	auto step := get_draw_subdivision_step(camera_position, i);
		float step = 1.0;
		float t = 0.0;

		// sample out the catmull spline into points and tangents
		while (t <= 1.0) {
			auto current_result = _catmull_interpolate(p0, p1, p2, p3, 0.0, t);
			auto current_position = current_result.first;
			auto current_tangent = current_result.second;

			// store sampled position and tangent; compute a stable frame after sampling
			current_position -= global_position;
			// Transform3D for each point.
			// Tangent is Y
			// Normal is X, calculated in parallel transport
			// Binormal is Z, calculated in parallel transport
			frames.push_back(Transform3D(Basis(Vector3(0, 0, 0), current_tangent, Vector3(0, 0, 0)), current_position));
			t += step;
		}
	}

	if (frames.size()) {
		_compute_parallel_transport(frames);

		// tube parameters. set the number of sides based on rope width
		// A 0.1 thickness rope will have 6 sides
		// A 1.0 thickness rope will have 12 sides
		// float ratio = Math::ease(rope_width, .2);
		float ratio = 0.5;
		int sides = Math::clamp(int(Math::lerp(3, 12, ratio)), 3, 12);
		auto radius = rope_width * 0.5;

		// override
		if (rope_sides >= 3) {
			sides = rope_sides;
		}

		PackedVector3Array verts;
		PackedVector3Array norms;
		PackedVector2Array uv1s;

		_emit_tube(frames, sides, radius, verts, norms, uv1s);

		/*
		// add end caps as a separate triangles surface to close the rope ends
		auto cap_scale := Vector3(rope_width, rope_width, rope_width);
		if (startcap_mesh && startcap_mesh.visible)
			{startcap_mesh.position = frames[0].origin;
			startcap_mesh.basis = frames[0].basis;
			startcap_mesh.scale = cap_scale;}
		else
			{_emit_endcap(true, frames[0], sides, radius);}

		auto idx = frames.size() - 1;
		if (endcap_mesh && endcap_mesh.visible)
			{endcap_mesh.position = frames[idx].origin;

			// flip the normal to mirror the orientation
			auto end_basis := frames[idx].basis;
			end_basis.x *= -1;
			endcap_mesh.basis = end_basis;

			endcap_mesh.scale = cap_scale;}
		else
			{_emit_endcap(false, frames[idx], sides, radius);}
		*/

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

PackedVector3Array VerletRope::_get_simulation_particles(int index) {
	auto segment_length = _get_average_segment_length();

	PackedVector3Array p;
	p.resize(4);

	if (index == 0)
		p[0] = _particles[index].pos_cur - (_particles[index].T * segment_length);
	else
		p[0] = _particles[index - 1].pos_cur;

	p[1] = _particles[index].pos_cur;
	p[2] = _particles[index + 1].pos_cur;

	if (index == simulation_particles - 2)
		p[3] = _particles[index + 1].pos_cur + (_particles[index + 1].T * segment_length);
	else
		p[3] = _particles[index + 2].pos_cur;

	return p;
}

Pair<Vector3, Vector3> VerletRope::_catmull_interpolate(const Vector3 &p0, const Vector3 &p1, const Vector3 &p2, const Vector3 &p3, float tension, float t) {
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

float VerletRope::get_current_rope_length() const {
	float length = 0.0;
	for (int i = 0; i < _particles.size() - 1; i++)
		length += (_particles[i + 1].pos_cur - _particles[i].pos_cur).length();
	return length;
}

float VerletRope::_get_average_segment_length() const {
	return rope_length / float(_particles.size() - 1);
}

void VerletRope::_stiff_rope() {
	for (int j = 0; j < stiffness_iterations; j++) {
		for (int i = 0; i < _particles.size() - 1; i++) {
			Particle &p0 = _particles[i];
			Particle &p1 = _particles[i + 1];

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

			// _particles[i] = p0;
			// _particles[i + 1] = p1;
		}
	}
}

void VerletRope::_verlet_process(float delta) {
	for (Particle &p : _particles) {
		if (p.attached)
			continue;

		Vector3 position_current_copy = p.pos_cur;
		p.pos_cur = (2.0 * p.pos_cur) - p.pos_prev + (delta * delta * p.accel);
		p.pos_prev = position_current_copy;
	}
}

void VerletRope::_apply_forces() {
	for (Particle &p : _particles) {
		Vector3 total_acceleration = Vector3(0, 0, 0);

		if (apply_gravity) {
			total_acceleration += gravity * gravity_scale;
		}

		if (apply_wind && wind_noise != nullptr) {
			auto timed_position = p.pos_cur + Vector3(1, 1, 1) * _time;
			auto wind_force = wind_noise->get_noise_3d(timed_position.x, timed_position.y, timed_position.z);
			total_acceleration += wind_scale * wind * wind_force;
		}
		if (apply_damping) {
			auto velocity = p.pos_cur - p.pos_prev;
			auto drag = -damping_factor * velocity.length() * velocity;
			total_acceleration += drag;
		}

		p.accel = total_acceleration;
	}
}

void VerletRope::_apply_constraints() {
	_stiff_rope();
}
