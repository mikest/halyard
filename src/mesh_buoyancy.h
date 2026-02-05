// MeshBuoyancy
//
// A utility class for calculating buoyancy forces from a submerged mesh volume.
//
// This class encapsulates mesh-based buoyancy calculations using triangle
// clipping against a liquid surface to determine submerged volume and centroid.

#pragma once

#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/variant/vector4.hpp>
#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/templates/hash_map.hpp>

using namespace godot;

class LiquidArea;

namespace halyard {

// Utility class for calculating buoyancy forces from a submerged mesh volume.
//
// Clips a mesh against a liquid surface to compute the submerged
// volume and centroid, then calculates the resulting buoyancy force.
//
// Typical usage:
// 1. Set the buoyancy_mesh and liquid_area
// 2. Call update_statics() once (or when mesh changes) to calculate total volume and centroid
// 3. Call update_dynamics() each physics frame to compute submerged volume and centroid
// 4. Call update_forces() to calculate the buoyancy force vector
class MeshBuoyancy {
private:
	// Configuration
	LiquidArea* _liquid_area = nullptr;
	Ref<ArrayMesh> _buoyancy_mesh;
	bool _ignore_waves = false;
	float _buoyancy = 1.0f;
	float _mass = 0.0f;

	// Static mesh properties (calculated once by update_statics)
	PackedVector3Array _vertex;
	float _mesh_volume = 0.0f;
	Vector3 _mesh_centroid = Vector3(0, 0, 0);
	float _sign = -1.0f;

	// Dynamic properties (calculated each frame by update_dynamics)
	float _submerged_volume = 0.0f;
	Vector3 _submerged_centroid = Vector3(0, 0, 0);
	Vector3 _buoyancy_normal = Vector3(0, 0, 0);

	// Force output (calculated by update_forces)
	Vector3 _buoyancy_force = Vector3(0, 0, 0);
	Vector3 _force_position = Vector3(0, 0, 0);

	// Depth cache for deduplication
	PackedFloat32Array _depths;
	HashMap<Vector3, Transform3D> _depth_map;

	// Debug support
	PackedVector3Array _submerged_verts;

	// Triangle calculations
	Vector4 _tri_contribution(const Vector3 &a, const Vector3 &b, const Vector3 &c, const Vector3 &o) const;
	Vector3 _intersect(const Vector3 &v1, const Vector3 &v2, float d1, float d2) const;
	Vector4 _partial_intersection(const Vector3 &a, const Vector3 &b, const Vector3 &c, const Vector3 &o,
			float _a, float _b, float _c, bool keep_below = true);

public:
	MeshBuoyancy() = default;

	// Configuration Accessors

	void set_liquid_area(LiquidArea* p_liquid_area);
	LiquidArea* get_liquid_area() const;

	void set_buoyancy_mesh(const Ref<ArrayMesh>& p_mesh);
	Ref<ArrayMesh> get_buoyancy_mesh() const;

	void set_ignore_waves(bool p_ignore);
	bool get_ignore_waves() const;

	// Buoyancy is a scalar. A value of 1.0 means neutral buoyancy (displaces its own weight in fluid).
    // if Buoyancy is INFINITY, then the buoyancy force is calculated solely from the submerged volume and liquid density, ignoring the buoyancy scalar.
	void set_buoyancy(float p_buoyancy);
	float get_buoyancy() const;

	// Mass in kg of the object. Used with buoyancy scalar to compute force.
	void set_mass(float p_mass);
	float get_mass() const;

	// Static Properties (Read-only)
	float get_mesh_volume() const;
	Vector3 get_mesh_centroid() const;
	const PackedVector3Array& get_vertices() const;

	// Dynamic Properties (Read-only)
	float get_submerged_volume() const;
	Vector3 get_submerged_centroid() const;
	Vector3 get_buoyancy_normal() const;
	float get_submerged_ratio() const;

	// Force Output (Read-only)

	// Buoyancy force including current effects. Calculated by update_forces().
	Vector3 get_buoyancy_force() const;

	// Global position where the force should be applied (submerged centroid).
	Vector3 get_force_position() const;

	//Debug

	// Submerged triangle vertices for debug visualization.
	const PackedVector3Array& get_submerged_verts() const;

	// Core Calculation Methods

	// Calculate static mesh properties (volume, centroid, vertices).
	// Call once after setting the buoyancy mesh, or whenever the mesh changes.
	void update_statics(const Transform3D& collider_transform);

	// Calculate dynamic submerged properties each frame.
	// Clips the mesh against the liquid surface to determine submerged volume and centroid.
	void update_dynamics(const Transform3D& collider_global_transform, const Transform3D& collider_local_transform);

	// Calculate buoyancy force from current submerged state.
	// Call update_dynamics() first to ensure submerged data is current.
	void update_forces(const Transform3D& body_transform, const Vector3& gravity);
};

} // namespace halyard
