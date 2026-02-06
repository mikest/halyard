/* BuoyancyMaterial
 *
 * A Resource that holds buoyancy-related physics properties for a body.
 *
 * Copyright (c) M. Estee
 * MIT License.
 */

#pragma once

#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/core/binder_common.hpp>

using namespace godot;

// Holds buoyancy physics properties such as density, volume, and drag.
// Attach to a Buoyancy node to define how a body interacts with liquid.
class BuoyancyMaterial : public Resource {
	GDCLASS(BuoyancyMaterial, Resource)

private:
	// Buoyancy scalar. 0 = disabled, 1.0 = neutral buoyancy (displaces its own
	// weight in fluid), >1.0 = positively buoyant.
	float _buoyancy = 2.0f;

	// Drag coefficients applied to the body while submerged.
	float _linear_drag = 3.0f;
	float _angular_drag = 1.0f;

	// Per-axis drag multipliers in body-local space.
	Vector3 _linear_drag_scale = Vector3(1.0f, 1.0f, 1.0f);
	Vector3 _angular_drag_scale = Vector3(1.0f, 1.0f, 1.0f);

protected:
	static void _bind_methods();

public:
	BuoyancyMaterial() = default;

	void set_buoyancy(float p_buoyancy);
	float get_buoyancy() const;

	void set_linear_drag(float p_drag);
	float get_linear_drag() const;

	void set_angular_drag(float p_drag);
	float get_angular_drag() const;

	void set_linear_drag_scale(const Vector3 &p_scale);
	Vector3 get_linear_drag_scale() const;

	void set_angular_drag_scale(const Vector3 &p_scale);
	Vector3 get_angular_drag_scale() const;
};
