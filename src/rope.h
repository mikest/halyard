#pragma once

#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/geometry_instance3d.hpp>

#include <godot_cpp/classes/fast_noise_lite.hpp>
#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/wrapped.hpp>
#include <godot_cpp/templates/local_vector.hpp>
#include <godot_cpp/templates/pair.hpp>
#include <godot_cpp/variant/typed_array.hpp>
#include <godot_cpp/variant/variant.hpp>

#include "property_utils.h"
#include "rope_appearance.h"
#include "rope_positions.h"

using namespace godot;

class Rope : public GeometryInstance3D {
	GDCLASS(Rope, GeometryInstance3D)

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

	float _rope_length = 4.0;
	Ref<RopeAppearance> _appearance;

	// simulation parameters
	bool _simulate = true;
	float _simulation_rate = 60.0;
	int _stiffness_iterations = 2;
	float _stiffness = 0.9;

	// attachments
	NodePath _start_anchor = ".";
	Ref<RopePositions> _anchors;
	NodePath _end_anchor;

	// forces
	bool _apply_wind = false;
	float _wind_scale = 20.0;
	Vector3 _wind = Vector3(1, 0, 0);
	Ref<FastNoiseLite> _wind_noise = nullptr;

	bool _apply_gravity = true;
	Vector3 _gravity = Vector3(0, -9.8, 0);
	float _gravity_scale = 1.0;

	bool _apply_damping = true;
	float _damping_factor = 100.0;

	// initial simulation
	int _preprocess_iterations = 5;
	float _preprocess_delta = 1 / 30.0;

protected:
	static void _bind_methods();

	void _notification(int p_what);

	// Particle list
	void _rebuild_rope();
	void _build_particles(const Vector3 &end_location, const Vector3 &global_position, const Vector3 &initial_accel, int particle_count, float segment_length);
	void _compute_parallel_transport(LocalVector<Transform3D> &frames) const;
	void _compute_particle_normals();
	void _calculate_frames_for_particles(LocalVector<Transform3D> &frames) const;
	PackedVector3Array _get_control_points_for_particle(int index) const;
	Pair<Vector3, Vector3> _catmull_interpolate(const Vector3 &p0, const Vector3 &p1, const Vector3 &p2, const Vector3 &p3, float tension, float t) const;

	void _emit_tube(LocalVector<Transform3D> &frames, int start, int end, int sides, float radius, PackedVector3Array &V, PackedVector3Array &N, PackedVector2Array &UV1);
	void _emit_endcap(bool front, const Transform3D &frame, int sides, float radius, PackedVector3Array &V, PackedVector3Array &N, PackedVector2Array &UV1);

	void _align_attachment_node(const NodePath &path, Transform3D xform, float offset);
	int _frame_at_offset(const LocalVector<Transform3D> &frames, float offset, bool from_end) const;

	float get_current_rope_length() const;
	float _get_average_segment_length() const;

	// Anchors
	void _update_anchors();
	void _update_anchor(NodePath &anchor, float position);
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
	Rope();
	~Rope() override;

	void _ready(void) override;

	// void _process(double delta) override;
	void _physics_process(double delta) override;

	// derived properties
	uint64_t get_particle_count() const;
	uint64_t get_particle_count_for_length() const;
	Ref<ArrayMesh> get_baked_mesh() const;

	// Exported Properties
	PROPERTY_GET_SET(float, rope_length, _queue_rebuild())
	PROPERTY_GET_SET(Ref<RopeAppearance>, appearance, _queue_rebuild())

	// anchors
	PROPERTY_GET_SET(NodePath, start_anchor, _queue_rebuild())
	PROPERTY_GET_SET(Ref<RopePositions>, anchors, _queue_rebuild())
	PROPERTY_GET_SET(NodePath, end_anchor, _queue_rebuild())

	// appearance passthrough accessors
#define APPEARENCE_ACCESSOR(m_type, m_name, m_default) \
	m_type get_##m_name() const { return _appearance != nullptr ? _appearance->get_##m_name() : m_default; }
	APPEARENCE_ACCESSOR(float, rope_width, 0.125)
	int get_rope_sides() const;
	APPEARENCE_ACCESSOR(float, rope_twist, 1.0)
	APPEARENCE_ACCESSOR(int, rope_lod, 2)
	APPEARENCE_ACCESSOR(Ref<Material>, material, nullptr)
	APPEARENCE_ACCESSOR(NodePath, start_attachment, NodePath())
	APPEARENCE_ACCESSOR(float, start_offset, 0.0)
	APPEARENCE_ACCESSOR(NodePath, end_attachment, NodePath())
	APPEARENCE_ACCESSOR(float, end_offset, 0.0)
	APPEARENCE_ACCESSOR(Ref<RopePositions>, attachments, nullptr)
	APPEARENCE_ACCESSOR(int, particles_per_meter, 2)
#undef APPEARENCE_ACCESSOR

	// simulation parameters
	PROPERTY_GET_SET(bool, simulate, {})
	PROPERTY_GET_SET(float, simulation_rate, {})
	PROPERTY_GET_SET(int, stiffness_iterations, {})
	PROPERTY_GET_SET(float, stiffness, {})

	// forces
	PROPERTY_GET_SET(bool, apply_wind, {})
	PROPERTY_GET_SET(float, wind_scale, {})
	PROPERTY_GET_SET(Vector3, wind, {})
	PROPERTY_GET_SET(Ref<FastNoiseLite>, wind_noise, {})

	PROPERTY_GET_SET(bool, apply_gravity, {})
	PROPERTY_GET_SET(Vector3, gravity, {})
	PROPERTY_GET_SET(float, gravity_scale, {})

	PROPERTY_GET_SET(bool, apply_damping, {})
	PROPERTY_GET_SET(float, damping_factor, {})

	// initial simulation
	PROPERTY_GET_SET(int, preprocess_iterations, {})
	PROPERTY_GET_SET(float, preprocess_delta, {})
};