/* Buoyancy

A class for adding Buoyancy to a RigidBody3D.

This objects manages the physics interactions between itself and a single LiquidArea in the scene.
*/

#include <godot_cpp/classes/area3d.hpp>
#include <godot_cpp/classes/collision_shape3d.hpp>
#include <godot_cpp/classes/convex_polygon_shape3d.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/physics_server3d.hpp>
#include <godot_cpp/classes/rigid_body3d.hpp>
#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/mesh.hpp>

#include <godot_cpp/classes/wrapped.hpp>
#include <godot_cpp/core/binder_common.hpp>
#include <godot_cpp/core/gdvirtual.gen.inc>
#include <godot_cpp/templates/local_vector.hpp>
#include <godot_cpp/templates/pair.hpp>
#include <godot_cpp/templates/hash_map.hpp>
#include <godot_cpp/variant/typed_array.hpp>
#include <godot_cpp/variant/variant.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>

#include "property_utils.h"
#include "node_debug.h"

using namespace godot;

class LiquidArea;

// Buoyancy mode
enum BuoyancyMode {
	BUOYANCY_COLLIDER = 0,
	BUOYANCY_PROBES = 1,
};

class Buoyancy : public Node, protected NodeDebug {
	GDCLASS(Buoyancy, Node)

private:
	// Exported properties
    LiquidArea* _liquid_area = nullptr;
	CollisionShape3D* _collider = nullptr;
	Ref<ArrayMesh> _buoyancy_mesh;
	BuoyancyMode _buoyancy_mode = BUOYANCY_COLLIDER;
    
	bool _apply_forces = true;

	float _submerged_linear_drag = 1.0f;
	float _submerged_angular_drag = 1.0f;
	Vector3 _linear_drag_scale = Vector3(1.0f, 1.0f, 1.0f);
	Vector3 _angular_drag_scale = Vector3(1.0f, 1.0f, 1.0f);
	bool _ignore_waves = false;
	float _probe_buoyancy = 1.0f;

	bool _calculate_mass_properties = false;
	float _density = 500.0f;
	Vector3 _com_offset = Vector3();

	// Info properties (read-only)
	float _mass = -1.0f;
	float _volume = 0.0f;
	Vector3 _center_of_mass = Vector3(0, 0, 0);
	Vector3 _inertia = Vector3(0, 0, 0);

	// Internal calculation variables
	Vector3 _buoyancy_normal = Vector3(0, 0, 0);
	
	Vector3 _submerged_centroid = Vector3(0, 0, 0);
	float _submerged_volume = 0.0f;
	int _submerged_probe_count = 0;
	Vector3 _mesh_centroid = Vector3(0, 0, 0);
	float _mesh_volume = 0.0f;
	float _sign = -1.0f;
	float _buoyancy_time = 0.0;
	float _last_submerged_ratio = 0.0f;

	// statics need updating
	bool _dirty = true;

	// Triangle data
	PackedVector3Array _vertex;
	// Optional explicit probe points for point-based buoyancy
	PackedVector3Array _buoyancy_probes;
	PackedFloat32Array _depths;
	HashMap<Vector3, Transform3D> _depth_map;

	// Debugging
	PackedVector3Array _submerged_verts;
	void _create_debug_mesh() override;
	void _update_debug_mesh() override;
	void _destroy_debug_mesh() override;

protected:
	static void _bind_methods();
	void _notification(int p_what);
	void _update_configuration_warnings();

	// mark the statics as needing to be recalculated.
	inline void _set_dirty() { _dirty = true; }

public:
	Buoyancy();
	~Buoyancy() override;

	// overrides
	PackedStringArray _get_configuration_warnings() const override;

	// Property getters/setters
    void set_liquid_area(LiquidArea *p_area);
    LiquidArea* get_liquid_area() const;

	void set_collider(CollisionShape3D *p_collider);
	CollisionShape3D* get_collider() const;

	// Optionally apply the forces to the parent RigidBody3D
	void set_apply_forces(bool p_apply_forces);
	float get_apply_forces() const;

	// Just use a flat plane for the liquid. Considerably faster.
	void set_ignore_waves(bool p_ignore);
	bool get_ignore_waves() const;

	// Debug
	void set_show_debug(bool p_show);
	bool get_show_debug() const;

	void set_debug_color(const Color &p_color);
	Color get_debug_color() const;

	// Buoyancy probes
	void set_buoyancy_probes(const PackedVector3Array &p_probes);
	PackedVector3Array get_buoyancy_probes() const;

	void set_buoyancy_mode(BuoyancyMode p_mode);
	BuoyancyMode get_buoyancy_mode() const;


	// While submerged, apply drag proportional to the submerged volume.
	void set_submerged_linear_drag(float p_drag);
	float get_submerged_linear_drag() const;

	void set_submerged_angular_drag(float p_drag);
	float get_submerged_angular_drag() const;

	void set_linear_drag_scale(const Vector3 &p_scale);
	Vector3 get_linear_drag_scale() const;

	void set_angular_drag_scale(const Vector3 &p_scale);
	Vector3 get_angular_drag_scale() const;

	void set_probe_buoyancy(float p_buoyancy);
	float get_probe_buoyancy() const;


	// Calculate and apply the intertia and mass for the body when static properties are updated
	void set_calculate_mass_properties(bool p_calculate);
	bool get_calculate_mass_properties() const;

	// Set density, when volume is calculated this will also apply to mass
	void set_density(float p_density);
	float get_density() const;

	// Center of mass offset from calculated CoM
	void set_com_offset(Vector3 p_com_offset);
	Vector3 get_com_offset() const;


	// Static properties
	float get_volume() const;
	Vector3 get_centroid() const;

	// Dynamic properties. Always updated while processing is enabled in BUOYANCY_COLLIDER mode.
	float get_submerged_volume() const;
	Vector3 get_submerged_centroid() const;
	Vector3 get_buoyancy_normal() const;

	// Returns the proportion of submerged probes in BUOYANCY_PROBES mode, and the ratio of submerged_volume/volume in BUOYANCY_COLLIDER mode.
	float get_submerged_ratio() const;


	// Called after updating dynamics if apply_forces is true. (submerged_volume, etc.)
	// Can be manually called from process_physics() if desired.
	void apply_buoyancy_mesh_forces(RigidBody3D *body, float delta);

	// Apply buoyancy forces using probe points instead of mesh
	void apply_buoyancy_probe_forces(RigidBody3D *body, float delta);


	// Informational getters
	float get_mass() const;
	Vector3 get_center_of_mass() const;
	Vector3 get_inertia() const;

	float _get_buoyancy_time() const;

private:
	void _update_statics();
	void _update_dynamics();

	// Triangle calculations
	Vector4 _tri_contribution(const Vector3 &a, const Vector3 &b, const Vector3 &c, const Vector3 &o) const;
	Vector3 _intersect(const Vector3 &v1, const Vector3 &v2, float d1, float d2) const;
	Vector4 _partial_intersection(const Vector3 &a, const Vector3 &b, const Vector3 &c, const Vector3 &o,
			float _a, float _b, float _c, bool keep_below = true) const;
};

VARIANT_ENUM_CAST(BuoyancyMode);