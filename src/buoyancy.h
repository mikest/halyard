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
#include <godot_cpp/variant/typed_array.hpp>
#include <godot_cpp/variant/variant.hpp>
#include <godot_cpp/variant/dictionary.hpp>

using namespace godot;

class LiquidArea;

class Buoyancy : public Node {
	GDCLASS(Buoyancy, Node)

private:
	// Exported properties
    LiquidArea* _liquid_area = nullptr;
	CollisionShape3D* _collider = nullptr;
    
	float _density = 500.0f;
	float _mass_scale = 1.0f;
	float _com_offset = 0.0f;
	float _submerged_linear_drag = 1.0f;
	float _submerged_angular_drag = 1.0f;
	Vector3 _linear_drag_scale = Vector3(1.0f, 1.0f, 1.0f);
	Vector3 _angular_drag_scale = Vector3(1.0f, 1.0f, 1.0f);
	bool _calculate_center_of_mass = true;
	bool _ignore_waves = false;
	bool _enabled = true;

	// Info properties (read-only)
	float _mass = -1.0f;
	float _volume = 0.0f;
	Vector3 _center_of_mass = Vector3(0, 0, 0);
	Vector3 _inertia = Vector3(0, 0, 0);

	// Internal calculation variables
	Vector3 _buoyancy_normal = Vector3(0, 0, 0);
	Vector3 _submerged_centroid = Vector3(0, 0, 0);
	float _submerged_volume = 0.0f;
	Vector3 _gravity_centroid = Vector3(0, 0, 0);
	float _gravity_volume = 0.0f;
	Vector3 _mesh_centroid = Vector3(0, 0, 0);
	float _mesh_volume = 0.0f;
	float _sign = -1.0f;

	Ref<ArrayMesh> _debug_mesh;

	// Triangle data
	PackedVector3Array _vertex;
	PackedFloat32Array _depths;
	Dictionary _depth_map;

	// Debugging
	bool _show_debug = true;
	Color _color = Color(1, 1, 1);
	Transform3D _T;

protected:
	static void _bind_methods();
	void _notification(int p_what);

public:
	Buoyancy();
	~Buoyancy() override;

	// Property getters/setters
    void set_liquid_area(LiquidArea *p_area);
    LiquidArea* get_liquid_area() const;

	void set_collider(CollisionShape3D *p_collider);
	CollisionShape3D* get_collider() const;

	void set_density(float p_density);
	float get_density() const;

	void set_mass_scale(float p_mass_scale);
	float get_mass_scale() const;

	void set_com_offset(float p_com_offset);
	float get_com_offset() const;

	void set_submerged_linear_drag(float p_drag);
	float get_submerged_linear_drag() const;

	void set_submerged_angular_drag(float p_drag);
	float get_submerged_angular_drag() const;

	void set_linear_drag_scale(const Vector3 &p_scale);
	Vector3 get_linear_drag_scale() const;

	void set_angular_drag_scale(const Vector3 &p_scale);
	Vector3 get_angular_drag_scale() const;

	void set_calculate_center_of_mass(bool p_calculate);
	bool get_calculate_center_of_mass() const;

	void set_ignore_waves(bool p_ignore);
	bool get_ignore_waves() const;

	void set_enabled(bool p_enabled);
	bool get_enabled() const;

	// Info getters
	float get_mass() const;
	float get_volume() const;
	Vector3 get_center_of_mass() const;
	Vector3 get_inertia() const;

private:
	void _update_statics();
	void _update_dynamics();
	void _property_changed(const String &prop);
	void _on_config_changed();
	void _apply_buoyancy_forces(RigidBody3D *body, float delta);

	// Triangle calculations
	Vector4 _tri_contribution(const Vector3 &a, const Vector3 &b, const Vector3 &c, const Vector3 &o) const;
	Vector3 _intersect(const Vector3 &v1, const Vector3 &v2, float d1, float d2) const;
	Vector4 _partial_intersection(const Vector3 &a, const Vector3 &b, const Vector3 &c, const Vector3 &o,
			float _a, float _b, float _c, bool keep_below = true) const;
};