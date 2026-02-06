/* LiquidArea

A class for representing a region that objects can float in.

The area Transform represents a plane where the liquid is considered to be in the -Y direction.
Any Buoyancy system is considered to be submerged if it is below the origin in the Y axis.

A derived class can override get_transforms_for_points to provide a more complex liquid surface, such as waves.
*/

#include <godot_cpp/classes/area3d.hpp>

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/physics_server3d.hpp>

#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/wrapped.hpp>
#include <godot_cpp/core/binder_common.hpp>
#include <godot_cpp/core/gdvirtual.gen.inc>
#include <godot_cpp/variant/typed_array.hpp>

#include "node_debug.h"
#include "property_utils.h"

using namespace godot;

class RigidBuoyancy;

class LiquidArea : public Node3D, protected NodeDebug {
	GDCLASS(LiquidArea, Node3D)

	float _density = 1000.0f; // kg/m^3
	Vector3 _current_speed = Vector3(0, 0, 0); // ms

	// Debug visualization

	Vector<Transform3D> _sampled_transforms;

protected:
	static void _bind_methods();
	void _notification(int p_what);

	void _destroy_debug_mesh() override;
	void _update_debug_mesh() override;

public:
	LiquidArea();
	~LiquidArea() override;

	// static accessor for finding the liquid area for a given scene tree
	static LiquidArea *get_liquid_area(SceneTree *p_tree);

	// Properties
	PROPERTY_GET_SET(float, density, {})
	PROPERTY_GET_SET(Vector3, current_speed, {})

	// debug visualization
	void set_show_debug(bool p_show);
	bool get_show_debug() const;

	void set_debug_color(const Color &p_color);
	Color get_debug_color() const;

	// Returns true if the given global point is considered to be submerged in the liquid.
	bool is_point_submerged(const Vector3 &global_point) const;

	// Return the transform of the liquid surface at the given global point.
	Transform3D get_liquid_transform(const Vector3 &global_point) const;

	// Given a set of global points, fills in the transforms at those points according to the liquid surface.
	// By default, this just fills in the area transform for all points.
	// Returns true on success.
	GDVIRTUAL2C(update_transforms_for_points, const PackedVector3Array &, TypedArray<Transform3D>);
	virtual void update_transforms_for_points(const PackedVector3Array &global_points, TypedArray<Transform3D> r_transforms) const;

	// called at the end of the frame to clear the previous frames sampled transforms
	void _clear_sampled_transforms();
};