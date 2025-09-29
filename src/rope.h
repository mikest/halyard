#pragma once

#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/geometry_instance3d.hpp>

#include <godot_cpp/classes/collision_object3d.hpp>
#include <godot_cpp/classes/collision_shape3d.hpp>
#include <godot_cpp/classes/fast_noise_lite.hpp>
#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/physics_server3d.hpp>
#include <godot_cpp/classes/wrapped.hpp>
#include <godot_cpp/core/binder_common.hpp>
#include <godot_cpp/core/gdvirtual.gen.inc>
#include <godot_cpp/templates/local_vector.hpp>
#include <godot_cpp/templates/pair.hpp>
#include <godot_cpp/variant/typed_array.hpp>
#include <godot_cpp/variant/variant.hpp>

#include "property_utils.h"

using namespace godot;

class RopeAnchorsBase;
class RopeAttachmentsBase;
class RopeAppearance;

class Rope : public GeometryInstance3D {
	GDCLASS(Rope, GeometryInstance3D)

	Ref<ArrayMesh> _generated_mesh;
	RID _physics_body;
	LocalVector<RID> _instances;

	// Rope particle, segments connect between particles.
	struct Particle {
		Vector3 pos_prev = Vector3(0, 0, 0);
		Vector3 pos_cur = Vector3(0, 0, 0);
		Vector3 accel = Vector3(0, 0, 0);

		Vector3 T = Vector3(0, 0, 0);
		Vector3 N = Vector3(0, 0, 0);
		Vector3 B = Vector3(0, 0, 0);

		bool attached = false;
		RID shape;

		Particle() = default;
		~Particle() = default;
	};

	// internal state
	LocalVector<Particle> _particles; // the individual points in the simulation
	LocalVector<Transform3D> _frames; // the transform frame for each LOD point along the rope
	LocalVector<Transform3D> _links; // the transforms for the points between each particle, always N-1 in count.
	bool _rebuild = true;
	bool _dirty = true;
	double _time = 0.0;
	double _simulation_delta = 0.0;

	// rope geometry
	float _rope_length = 4.0;
	int _grow_from = Start;
	Ref<RopeAppearance> _appearance;

	// simulation parameters
	bool _simulate = true;
	float _simulation_rate = 60.0;
	int _stiffness_iterations = 2;
	float _stiffness = 0.9;
	float _friction = 0.5;

	// initial simulation
	int _preprocess_time = 1.0;
	bool _jitter_initial_position = true;

	int _collision_layer = 1;
	int _collision_mask = 1;

	// attachments
	NodePath _start_anchor = ".";
	Ref<RopeAnchorsBase> _anchors;
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

protected:
	static void _bind_methods();

	void _notification(int p_what);

	// Particle list
	void _build_particles(const Vector3 &end_location, const Vector3 &global_position, const Vector3 &initial_accel, int particle_count, float segment_length);
	void _compute_parallel_transport(LocalVector<Transform3D> &frames) const;
	void _compute_particle_normals();
	void _calculate_frames_for_particles(LocalVector<Transform3D> &frames) const;
	void _calculate_links_for_particles(LocalVector<Transform3D> &links) const;
	PackedVector3Array _get_control_points_for_particle(int index) const;
	Pair<Vector3, Vector3> _catmull_interpolate(const Vector3 &p0, const Vector3 &p1, const Vector3 &p2, const Vector3 &p3, float tension, float t) const;

	void _emit_tube(LocalVector<Transform3D> &frames, int start, int end, int sides, float radius, PackedVector3Array &V, PackedVector3Array &N, PackedVector2Array &UV1);
	void _emit_endcap(bool front, const Transform3D &frame, int sides, float radius, PackedVector3Array &V, PackedVector3Array &N, PackedVector2Array &UV1);

	void _align_attachment_node(const NodePath &path, Transform3D xform, float offset);
	int _frame_at_offset(const LocalVector<Transform3D> &frames, float offset, bool from_end) const;

	float get_current_rope_length() const;
	float _get_average_segment_length() const;

	int _get_index_for_position(float position) const;

	// Anchors
	void _update_anchors();
	void _update_anchor(NodePath &anchor, float position);
	bool _get_node_transform(const NodePath &path, Transform3D &xform) const;

	// Physics
	void _rebuild_rope();
	void _queue_rope_rebuild();
	bool _pop_rebuild();

	void _update_physics(float delta, int iterations);
	void _apply_chain_constraint(int from_idx);
	void _stiff_rope(int interations);
	void _verlet_process(float delta);
	void _apply_forces();
	void _apply_constraints();
	void _update_collision_shapes();

	void _clear_physics_shapes();
	void _rebuild_physics_shapes();
	void _prepare_physics_server();

	// Mesh building
	void _draw_rope();
	void _queue_redraw();
	bool _pop_is_dirty();
	void _clear_instances();
	void _rebuild_instances();

	void _on_appearance_changed();

public:
	enum From {
		Start = 0,
		End = 1,
	};

	Rope();
	Rope(const Rope &other);
	virtual ~Rope() override;

	void _internal_ready();
	void _internal_process(double delta);
	void _internal_physics_process(double delta);

	// derived properties
	uint64_t get_particle_count() const;
	uint64_t get_particle_count_for_length() const;
	TypedArray<Vector3> get_particle_positions() const;

	Ref<ArrayMesh> get_baked_mesh() const;

	// utility
	int get_rope_frame_count() const;
	Transform3D get_rope_frame(int index) const;
	TypedArray<Transform3D> get_all_rope_frames() const;

	// utility
	int get_link_count() const;
	Transform3D get_link(int index) const;
	TypedArray<Transform3D> get_all_links() const;

	// subclassing callbacks

	// These index ranges do not include the special cased start/end nodes.
	GDVIRTUAL0RC(int, _get_anchor_count);
	virtual int _get_anchor_count() const;

	GDVIRTUAL0RC(int, _get_attachment_count);
	virtual int _get_attachment_count() const;

	// Return the position of the anchor along the rope in the range [-1, 0..1]
	// If the position is -1, the anchor is disabled.
	GDVIRTUAL1RC(float, _get_anchor_position, int);
	virtual float _get_anchor_position(int idx) const;

	GDVIRTUAL1RC(Transform3D, _get_anchor_transform, int);
	virtual Transform3D _get_anchor_transform(int idx) const;

	// Return the position of the attachment along the rope in the range [-1, 0..1]
	// If the position is -1, the attachment is unattached.
	GDVIRTUAL1RC(float, _get_attachment_position, int);
	virtual float _get_attachment_position(int idx) const;

	GDVIRTUAL1RC(NodePath, _get_attachment_nodepath, int);
	virtual NodePath _get_attachment_nodepath(int idx) const;

	GDVIRTUAL1RC(Transform3D, _get_attachment_transform, int);
	Transform3D _get_attachment_transform(int idx) const;

	// Exported Properties
	PROPERTY_GET_SET(float, rope_length, _queue_rope_rebuild())
	PROPERTY_GET_SET(int, grow_from, {})

	Ref<RopeAppearance> get_appearance() const;
	void set_appearance(const Ref<RopeAppearance> &val);

	// anchors
	PROPERTY_GET_SET(NodePath, start_anchor, _queue_redraw())
	PROPERTY_GET_SET(NodePath, end_anchor, _queue_redraw())

	void set_anchors(const Ref<RopeAnchorsBase> &);
	Ref<RopeAnchorsBase> get_anchors() const;

	// appearance passthrough accessors
#define APPEARENCE_ACCESSOR(m_type, m_name, m_default) \
	m_type get_##m_name() const;
	APPEARENCE_ACCESSOR(float, rope_width, 0.125)
	int get_rope_sides() const;
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

	// simulation parameters
	PROPERTY_GET_SET(bool, simulate, {})
	PROPERTY_GET_SET(float, simulation_rate, {})
	PROPERTY_GET_SET(int, stiffness_iterations, {})
	PROPERTY_GET_SET(float, stiffness, {})
	PROPERTY_GET_SET(float, friction, {})

	// initial simulation
	PROPERTY_GET_SET(float, preprocess_time, {})
	PROPERTY_GET_SET(bool, jitter_initial_position, {})

	int get_collision_layer() const;
	void set_collision_layer(int layer);
	int get_collision_mask() const;
	void set_collision_mask(int mask);

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
};

VARIANT_ENUM_CAST(Rope::From);