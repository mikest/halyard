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

class FastRope : public GeometryInstance3D {
	GDCLASS(FastRope, GeometryInstance3D)

	Ref<ArrayMesh> _generated_mesh;

	// Rope particle, segments connect between particles.
	struct Particle {
		Vector3 pos_prev = Vector3(0, 0, 0);
		Vector3 pos_cur = Vector3(0, 0, 0);
		Vector3 accel = Vector3(0, 0, 0);

		Vector3 T = Vector3(0, 0, 0);
		Vector3 N = Vector3(0, 0, 0);
		Vector3 B = Vector3(0, 0, 0);

		bool attached = false;

		Particle() = default;
		~Particle() = default;
	};

	// internal state
	LocalVector<Particle> _particles;
	bool _dirty = true;
	double _time = 0.0;
	double _simulation_delta = 0.0;

	// rope geometry
	int rope_particles = 10;
	float rope_width = 0.25;
	float rope_length = 4.0;
	int rope_sides = 6;
	float rope_twist = 1.0;
	int rope_lod = 1;
	Ref<Material> material = nullptr;
	NodePath start_cap;
	NodePath end_cap;

	// simulation parameters
	bool simulate = true;
	float simulation_rate = 60.0;
	int stiffness_iterations = 2;
	float stiffness = 0.9;

	// attachments
	NodePath start_anchor = ".";
	NodePath mid_anchor;
	int mid_index;
	NodePath end_anchor;

	// forces
	bool apply_wind = false;
	float wind_scale = 20.0;
	Vector3 wind = Vector3(1, 0, 0);
	Ref<FastNoiseLite> wind_noise = nullptr;

	bool apply_gravity = true;
	Vector3 gravity = Vector3(0, -9.8, 0);
	float gravity_scale = 1.0;

	bool apply_damping = true;
	float damping_factor = 100.0;

	// initial simulation
	int preprocess_iterations = 5;
	float preprocess_delta = 1 / 30.0;

protected:
	static void _bind_methods();

	void _notification(int p_what);

	// Particle list
	void _create_rope();
	void _build_particles(const Vector3 &end_location, const Vector3 &global_position, const Vector3 &initial_accel, int particle_count, float segment_length);
	void _compute_parallel_transport(LocalVector<Transform3D> &frames);
	void _compute_particle_normals();
	PackedVector3Array _get_simulation_particles(int index);
	Pair<Vector3, Vector3> _catmull_interpolate(const Vector3 &p0, const Vector3 &p1, const Vector3 &p2, const Vector3 &p3, float tension, float t);

	void _emit_tube(LocalVector<Transform3D> &frames, int sides, float radius, PackedVector3Array &V, PackedVector3Array &N, PackedVector2Array &UV1);
	void _emit_endcap(bool front, const Transform3D &frame, int sides, float radius, PackedVector3Array &V, PackedVector3Array &N, PackedVector2Array &UV1);
	void _align_cap_node(const NodePath &path, Transform3D xform);

	float get_current_rope_length() const;
	float _get_average_segment_length() const;

	// Anchors
	void _update_anchor(NodePath &anchor, int index);
	bool _get_anchor_transform(const NodePath &path, Transform3D &xform) const;

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
	FastRope();
	~FastRope() override;

	void _ready(void) override;

	// void _process(double delta) override;
	void _physics_process(double delta) override;

	void set_particle_count(uint64_t p_count);
	uint64_t get_particle_count() const;

	Ref<ArrayMesh> get_baked_mesh() const;

// Getter/Setter macros for properties
#define PROPERTY_GET(m_type, m_prop) \
	const m_type &get_##m_prop() const { return m_prop; }
#define PROPERTY_SET(m_type, m_prop, m_update) \
	void set_##m_prop(const m_type &val) {     \
		if (val != m_prop) {                   \
			m_prop = val;                      \
			m_update;                          \
		}                                      \
	}
#define PROPERTY_GET_SET(m_type, m_prop, m_update) \
	PROPERTY_GET(m_type, m_prop)                   \
	PROPERTY_SET(m_type, m_prop, m_update)

	// Exported Properties
	PROPERTY_GET_SET(int, rope_particles, _create_rope())
	PROPERTY_GET_SET(float, rope_width, _queue_rebuild())
	PROPERTY_GET_SET(float, rope_length, _queue_rebuild())
	PROPERTY_GET_SET(int, rope_sides, _queue_rebuild())
	PROPERTY_GET_SET(float, rope_twist, _queue_rebuild())
	PROPERTY_GET_SET(int, rope_lod, _queue_rebuild())

	void set_material(const Ref<Material> &p_material);
	Ref<Material> get_material() const;

	PROPERTY_GET_SET(NodePath, start_cap, {})
	PROPERTY_GET_SET(NodePath, end_cap, {})

	// simulation parameters
	PROPERTY_GET_SET(bool, simulate, {})
	PROPERTY_GET_SET(float, simulation_rate, {})
	PROPERTY_GET_SET(int, stiffness_iterations, {})
	PROPERTY_GET_SET(float, stiffness, {})

	// anchors
	PROPERTY_GET_SET(NodePath, start_anchor, _queue_rebuild())
	PROPERTY_GET_SET(NodePath, mid_anchor, _queue_rebuild())
	PROPERTY_GET_SET(int, mid_index, _queue_rebuild())
	PROPERTY_GET_SET(NodePath, end_anchor, _queue_rebuild())

	// forces
	PROPERTY_GET_SET(bool, apply_wind, {})
	PROPERTY_GET_SET(float, wind_scale, {})
	PROPERTY_GET_SET(Vector3, wind, {})

	void set_wind_noise(const Ref<FastNoiseLite> &p_noise);
	Ref<FastNoiseLite> get_wind_noise() const;

	PROPERTY_GET_SET(bool, apply_gravity, {})
	PROPERTY_GET_SET(Vector3, gravity, {})
	PROPERTY_GET_SET(float, gravity_scale, {})

	PROPERTY_GET_SET(bool, apply_damping, {})
	PROPERTY_GET_SET(float, damping_factor, {})

	// initial simulation
	PROPERTY_GET_SET(int, preprocess_iterations, {})
	PROPERTY_GET_SET(float, preprocess_delta, {})
};

#undef PROPERTY_GET
#undef PROPERTY_SET
#undef PROPERTY_GET_SET