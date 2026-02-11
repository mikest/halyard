/* RigidBuoyancy

A class for adding Buoyancy to a RigidBody3D.

This objects manages the physics interactions between itself and a single LiquidArea in the scene.
*/

#include <godot_cpp/classes/collision_shape3d.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/rigid_body3d.hpp>

#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/wrapped.hpp>
#include <godot_cpp/core/binder_common.hpp>
#include <godot_cpp/core/gdvirtual.gen.inc>
#include <godot_cpp/variant/typed_array.hpp>
#include <godot_cpp/variant/variant.hpp>

#include "buoyancy_material.h"
#include "mesh_buoyancy.h"
#include "node_debug.h"
#include "probe_buoyancy.h"
#include "property_utils.h"

using namespace godot;
using namespace halyard;

class LiquidArea;

// Buoyancy mode
enum BuoyancyMode {
	BUOYANCY_COLLIDER = 0,
	BUOYANCY_PROBES = 1,
};

class RigidBuoyancy : public Node, protected NodeDebug {
	GDCLASS(RigidBuoyancy, Node)

private:
	// Exported properties
	LiquidArea *_liquid_area = nullptr;
	CollisionShape3D *_collider = nullptr;
	BuoyancyMode _buoyancy_mode = BUOYANCY_COLLIDER;

	bool _apply_forces = true;
	Ref<BuoyancyMaterial> _buoyancy_material;
	ProbeBuoyancy _probe_buoyancy;
	MeshBuoyancy _mesh_buoyancy;

	bool _ignore_waves = false;
	bool _use_buoyancy_scalar = true;

	bool _calculate_mass_properties = false;
	float _density = 500.0f;
	Vector3 _com_offset = Vector3();

	// Info properties (read-only)
	float _mass = -1.0f;
	float _volume = 0.0f;
	Vector3 _center_of_mass = Vector3(0, 0, 0);
	Vector3 _inertia = Vector3(0, 0, 0);

	float _buoyancy_time = 0.0;
	float _last_submerged_ratio = 0.0f;
	float _submerged_threshold = 0.0f;

	// statics need updating
	bool _dirty = true;
	void _create_debug_mesh() override;
	void _update_debug_mesh() override;
	void _destroy_debug_mesh() override;

	void _update_last_probe_transforms();

protected:
	static void _bind_methods();
	void _notification(int p_what);
	void _update_configuration_warnings();

	// mark the statics as needing to be recalculated.
	inline void _set_dirty() { _dirty = true; }

public:
	RigidBuoyancy();
	~RigidBuoyancy() override;

	// overrides
	PackedStringArray _get_configuration_warnings() const override;

	// Property getters/setters
	void set_liquid_area(LiquidArea *p_area);
	LiquidArea *get_liquid_area() const;

	void set_collider(CollisionShape3D *p_collider);
	CollisionShape3D *get_collider() const;

	// Material resource containing buoyancy physics properties
	void set_buoyancy_material(const Ref<BuoyancyMaterial> &p_material);
	Ref<BuoyancyMaterial> get_buoyancy_material() const;

	// Optionally apply the forces to the parent RigidBody3D
	void set_apply_forces(bool p_apply_forces);
	float get_apply_forces() const;

	// Just use a flat plane for the liquid. Considerably faster.
	void set_ignore_waves(bool p_ignore);
	bool get_ignore_waves() const;

	// Use the buoyancy scalar on MeshBuoyancy/ProbeBuoyancy instead of raw Archimedes
	void set_use_buoyancy_scalar(bool p_use);
	bool get_use_buoyancy_scalar() const;

	// Debug
	void set_show_debug(bool p_show);
	bool get_show_debug() const;

	void set_debug_color(const Color &p_color);
	Color get_debug_color() const;

	// Buoyancy probes
	void set_probes(const PackedVector3Array &p_probes);
	PackedVector3Array get_probes() const;

	void set_buoyancy_mode(BuoyancyMode p_mode);
	BuoyancyMode get_buoyancy_mode() const;

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

	void set_submerged_threshold(float threshold);
	float get_submerged_threshold() const;

	bool get_is_submerged() const;

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
};

VARIANT_ENUM_CAST(BuoyancyMode);