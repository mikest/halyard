#pragma once

#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/geometry_instance3d.hpp>

#include <godot_cpp/classes/collision_object3d.hpp>
#include <godot_cpp/classes/collision_shape3d.hpp>
#include <godot_cpp/classes/fast_noise_lite.hpp>
#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/physics_ray_query_parameters3d.hpp>
#include <godot_cpp/classes/physics_server3d.hpp>
#include <godot_cpp/classes/wrapped.hpp>
#include <godot_cpp/core/binder_common.hpp>
#include <godot_cpp/core/gdvirtual.gen.inc>
#include <godot_cpp/templates/hash_map.hpp>
#include <godot_cpp/templates/local_vector.hpp>
#include <godot_cpp/templates/pair.hpp>
#include <godot_cpp/variant/typed_array.hpp>
#include <godot_cpp/variant/variant.hpp>

#include "property_utils.h"
#include "rope_anchor.h"
#include "rope_mesh.h"

using namespace godot;

class RopeAnchor;
class RopeAnchorsBase;
class RopeAppearance;
class LiquidArea;


class Rope : public GeometryInstance3D {
	GDCLASS(Rope, GeometryInstance3D)

public:
	enum Distribution {
		ABSOLUTE=0,  // Distance is measured from the start or end of the rope.
		RELATIVE,    // Distance is a relative offset from previous anchor. Offsets can be negative.
		UNIFORM,     // Anchors are distributed uniformly along the rope. Offsets are ignored.
		SCALAR,      // Distance is a scalar multiple of the rope length, where 0=start and 1=end
		REAL,		 // Distance is auto calculated from the initial anchor transforms.
	};

	enum From {
		Start = 0,
		End = 1,
	};

private:
	Ref<RopeMesh> _rope_mesh;
	RID _physics_body;
	LocalVector<RID> _instances;
	LiquidArea *_liquid_area = nullptr;

	// Anchor point along the rope.
	struct Anchor {
		float offset = 0.0; // Distance in meters along rope. positive from start.
		bool from_end = false;	// Distance is measured from end instead of start.
		Transform3D transform = Transform3D(); // The transform of the anchor in world space
		AnchorBehavior behavior = AnchorBehavior::ANCHORED;
		RigidBody3D *rigid_body = nullptr; // RigidBody to apply forces to
		Node3D *node = nullptr; // If using a node for an anchor, transform and behavior may be derrived from it.
		NodePath node_path = "";
		float _abs_offset = 0.0; // Absolute offset along the rope, calculated from start

		Anchor() = default;
		Anchor(float p, Transform3D t, AnchorBehavior b = AnchorBehavior::ANCHORED, RigidBody3D *rb = nullptr, Node3D *n = nullptr) :
				offset(p), transform(t), behavior(b), rigid_body(rb), node(n) {}

		bool operator<(const Anchor &other) const {
			return _abs_offset < other._abs_offset;
		}

		bool operator==(const Anchor &other) const {
			return offset == other.offset && transform == other.transform && behavior == other.behavior && rigid_body == other.rigid_body && node == other.node;
		}
	};

	// Rope particle, segments connect between particles.
	struct Particle {
		Vector3 pos_prev = Vector3(0, 0, 0);
		Vector3 pos_cur = Vector3(0, 0, 0);
		Vector3 accel = Vector3(0, 0, 0);

		Vector3 T = Vector3(0, 0, 0);
		Vector3 N = Vector3(0, 0, 0);
		Vector3 B = Vector3(0, 0, 0);

		int64_t anchor_idx = -1;
		float stretch = 0.0;

		RID shape;

		Particle() = default;
		~Particle() = default;
	};

	// internal state

	Distribution _anchor_distribution = Distribution::ABSOLUTE;
	LocalVector<Anchor> _anchors;
	LocalVector<Particle> _particles; // the individual points in the simulation
	LocalVector<Transform3D> _frames; // the transform frame for each LOD point along the rope
	LocalVector<Transform3D> _links; // the transforms for the points between each particle, always N-1 in count.
	bool _rebuild = true;
	bool _is_rebuilding = false;
	
	bool _rope_dirty = true;
	bool _anchors_dirty = true;
	bool _attachments_dirty = true;

	double _time = 0.0;
	double _simulation_delta = 0.0;

	// To avoid repeated heap allocations
	// We store these as class members are reuse each physics frame
	Ref<PhysicsRayQueryParameters3D> _ray_cast;
	TypedArray<RID> _exclusion_list;

	// rope geometry
	float _rope_length = 4.0;
	float _rope_width = 0.125;
	float _particles_per_meter = 2.0;
	int _grow_from = Start;
	Ref<RopeAppearance> _appearance;

	// simulation parameters
	bool _simulate = true;
	float _simulation_rate = 60.0;
	int _stiffness_iterations = 2;
	float _stiffness = 0.9;
	float _friction = 0.5;
	float _tension_force_scale = 1.0; // Multiplier for forces applied to attached bodies
	float _max_tension_force = 1000.0; // Maximum force magnitude to prevent instability

	// initial simulation
	int _preprocess_time = 1.0;
	bool _jitter_initial_position = true;

	int _collision_layer = 1;
	int _collision_mask = 1;

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

	bool _apply_buoyancy = false;
	float _buoyancy_scale = 1.0; // float on surface
	float _submerged_drag = 100.0; // crazy high drag when submerged

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

	void _align_attachment_node(const NodePath &path, Transform3D xform, float offset);
	int _frame_at_offset(const LocalVector<Transform3D> &frames, float offset, bool from_end) const;

	float get_current_rope_length() const;
	float _get_average_segment_length() const;

	int _get_index_for_position(float position) const;

	// Anchors Property List
	void _get_property_list(List<PropertyInfo> *p_list) const;
	bool _set(const StringName &p_name, const Variant &p_property);
	bool _get(const StringName &p_name, Variant &r_property) const;

	// Anchors
	void _internal_update_anchors();

	bool _is_anchor_free(int anchor_idx, int anchor_count) const;
	bool _is_anchor_fixed(int anchor_idx, int anchor_count) const;	// ANCHORED || GUIDED
	bool _is_anchor_moving(int anchor_idx, int anchor_count) const;	// SLIDING || TOWING
	bool _is_rope_sliding(int anchor_idx, int anchor_count) const;	// SLIDING || GUIDED

	// Physics
	void _rebuild_rope();
	void _rebuild_anchors();
	void _queue_rope_rebuild();
	bool _pop_rebuild();

	void _update_physics(float delta, int iterations);
	void _apply_chain_constraint(int from_idx);
	void _apply_anchor_forces(Particle &p_particle, int p_anchor_idx, const Vector3 &tension);

	void _stiff_rope(int interations);
	void _balance_tension();
	void _verlet_process(float delta);
	void _apply_forces();
	void _apply_constraints();
	bool _is_jolt_3d() const;
	void _update_collision_shapes();

	void _clear_physics_shapes();
	void _rebuild_physics_shapes();
	void _prepare_physics_server();

	// Mesh building
	void _update_aabb();
	float _lod_factor() const;
	void _draw_rope();
	void _queue_redraw();
	bool _pop_is_dirty();
	void _set_instances_visible(bool p_visible);
	void _clear_instances();
	void _rebuild_instances();

	void _on_appearance_changed();

public:

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

#pragma region Properties
	// length of rope in meters
	PROPERTY_GET_SET(float, rope_length, _queue_rope_rebuild())
	
	// rope diameter
	void set_rope_width(float val);
	float get_rope_width() const;
	
	// number of particles per meter in simulation
	void set_particles_per_meter(float val);
	float get_particles_per_meter() const;

	PROPERTY_GET_SET(int, grow_from, {})
	PROPERTY_GET_SET(bool, jitter_initial_position, {})
#pragma endregion

#pragma region Anchors
	// Anchor management
	void set_anchor_count(int count);
	int get_anchor_count() const;
	void clear_anchors();

	// Offset from start or end of rope
	void set_anchor_offset(int idx, float offset);
	float get_anchor_offset(int idx) const;

	// Change the end the offset will calculate from
	void set_anchor_from(int idx, int from);
	int get_anchor_from(int idx) const;

	// This will also set the transform, rigidbody, (and behavior if the node is a RopeAnchor)
	void set_anchor_nodepath(int idx, const NodePath &path);
	NodePath get_anchor_nodepath(int idx) const;

	// Set transform
	void set_anchor_transform(int idx, const Transform3D &transform);
	Transform3D get_anchor_transform(int idx) const;
	
	// Set anchor behavior
	void set_anchor_behavior(int idx, AnchorBehavior behavior);
	AnchorBehavior get_anchor_behavior(int idx) const;

	// Get the computed absolute offset from start of rope for the given anchor index.
	float get_anchor_abs_offset(int idx) const;

	// Anchor distribution mode
	void set_anchor_distribution(int val);
	int get_anchor_distribution() const;
	
	// Get the anchor rigidbody
	void set_anchor_rigidbody(int idx, RigidBody3D *body);
	RigidBody3D* get_anchor_rigidbody(int idx) const;
#pragma endregion

#pragma region Attachments
	// int get_attachment_count(int idx) const;
	// float get_attachment_position(int idx) const;
	// NodePath get_attachment_nodepath(int idx) const;
	// Transform3D get_attachment_transform(int idx) const;
#pragma endregion

#pragma region Subclassing
	void _notify_anchors_changed();
	void _notify_attachments_changed();

	// Subclasses can override these to provide dynamic anchor lists after a _notify_anchors_changed
	GDVIRTUAL0C(_update_anchors);
	void _update_anchors() const;

	GDVIRTUAL0C(_update_attachments);
	void _update_attachments() const;

	GDVIRTUAL1RC(Transform3D, _get_attachment_local_transform, int);
	Transform3D _get_attachment_local_transform(int attach_idx) const;
#pragma endregion

	Ref<RopeAppearance> get_appearance() const;
	void set_appearance(const Ref<RopeAppearance> &val);

	// appearance passthrough accessors
#define APPEARENCE_ACCESSOR(m_type, m_name, m_default) \
	m_type get_##m_name() const;
	int get_rope_sides() const;
	APPEARENCE_ACCESSOR(float, rope_twist, 1.0)
	APPEARENCE_ACCESSOR(int, rope_lod, 2)
	APPEARENCE_ACCESSOR(Ref<Material>, material, nullptr)
	APPEARENCE_ACCESSOR(float, start_offset, 0.0)
	APPEARENCE_ACCESSOR(float, end_offset, 0.0)
#undef APPEARENCE_ACCESSOR

	// simulation parameters
	PROPERTY_GET_SET(bool, simulate, {})
	PROPERTY_GET_SET(float, simulation_rate, {})
	PROPERTY_GET_SET(int, stiffness_iterations, {})
	PROPERTY_GET_SET(float, stiffness, {})
	PROPERTY_GET_SET(float, friction, {})
	PROPERTY_GET_SET(float, tension_force_scale, {})
	PROPERTY_GET_SET(float, max_tension_force, {})

	// initial simulation
	PROPERTY_GET_SET(float, preprocess_time, {})

	int get_collision_layer() const;
	void set_collision_layer(int layer);
	int get_collision_mask() const;
	void set_collision_mask(int mask);

	// forces
	PROPERTY_GET_SET(bool, apply_buoyancy, {});
	PROPERTY_GET_SET(float, buoyancy_scale, {});
	PROPERTY_GET_SET(float, submerged_drag, {})
	void set_liquid_area(LiquidArea *liquid_area);
	LiquidArea *get_liquid_area() const;

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
VARIANT_ENUM_CAST(Rope::Distribution);