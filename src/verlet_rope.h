#pragma once

#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/geometry_instance3d.hpp>

#include <godot_cpp/classes/fast_noise_lite.hpp>
#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/wrapped.hpp>
#include <godot_cpp/templates/local_vector.hpp>
#include <godot_cpp/templates/pair.hpp>
#include <godot_cpp/variant/variant.hpp>

using namespace godot;

class VerletRope : public GeometryInstance3D {
	GDCLASS(VerletRope, GeometryInstance3D)

	Ref<ArrayMesh> _generated_mesh;

	// Rope particle, segments connect between particles.
	struct Particle {
		Vector3 pos_prev = Vector3();
		Vector3 pos_cur = Vector3();
		Vector3 accel = Vector3();

		Vector3 T = Vector3();
		Vector3 N = Vector3();
		Vector3 B = Vector3();

		bool attached = false;

		Particle() = default;
		~Particle() = default;
	};

	LocalVector<Particle> _particles;
	bool _dirty = true;
	double _time = 0.0;
	double _simulation_delta = 0.0;

	float rope_width = 1.0;
	float rope_length = 2.0;
	int rope_sides = 6;
	float rope_twist = 0.25;

	bool attach_start = true;
	int preprocess_iterations = 1;
	float preprocess_delta = 1 / 60.0;

	bool simulate = true;
	int simulation_particles = 10;
	float simulation_rate = 60.0;
	int stiffness_iterations = 2;
	float stiffness = 0.9;

	bool apply_wind = false;
	float wind_scale = 20.0;
	Vector3 wind = Vector3(1, 0, 0);

	bool apply_gravity = true;
	// Vector3 gravity = Vector3(0, -9.8, 0);
	Vector3 gravity = Vector3(0, -0.1, 0);
	float gravity_scale = 1.0;

	bool apply_damping = true;
	float damping_factor = 100.0;

	Ref<Material> material = nullptr;
	Ref<FastNoiseLite> wind_noise = nullptr;

protected:
	static void _bind_methods();

	void _notification(int p_what);

	// Particle list
	void _create_rope();
	void _build_particles(const Vector3 &end_location, const Vector3 &global_position, const Vector3 &initial_accel, int particle_count, float segment_length);
	void _compute_parallel_transport(LocalVector<Transform3D> &frames);
	PackedVector3Array _get_simulation_particles(int index);
	Pair<Vector3, Vector3> _catmull_interpolate(const Vector3 &p0, const Vector3 &p1, const Vector3 &p2, const Vector3 &p3, float tension, float t);

	void _emit_tube(LocalVector<Transform3D> &frames, int sides, float radius, PackedVector3Array &V, PackedVector3Array &N, PackedVector2Array &UV1);
	void _emit_endcap(bool front, const Transform3D &frame, int sides, float radius, PackedVector3Array &V, PackedVector3Array &N, PackedVector2Array &UV1);

	float get_current_rope_length() const;
	float _get_average_segment_length() const;

	// Physics
	void _stiff_rope();
	void _verlet_process(float delta);
	void _apply_forces();
	void _apply_constraints();

	// Mesh building
	void _rebuild_mesh();
	void _queue_rebuild() { _dirty = true; }
	bool _pop_is_dirty() {
		bool is_dirty = _dirty;
		_dirty = false;
		return is_dirty;
	}

public:
	VerletRope();
	~VerletRope() override;

	void _ready(void) override;

	void _process(double delta) override;
	void _physics_process(double delta) override;

	void set_particle_count(uint64_t p_count);
	uint64_t get_particle_count() const;

	Ref<ArrayMesh> get_baked_mesh() const;

	void VerletRope::set_material(const Ref<Material> &p_material);
	Ref<Material> VerletRope::get_material() const;
};
