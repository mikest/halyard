#pragma once

#include <godot_cpp/classes/character_body3d.hpp>
#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/core/binder_common.hpp>
#include <godot_cpp/variant/basis.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "buoyancy_material.h"
#include "node_debug.h"
#include "probe_buoyancy.h"

using namespace godot;
using namespace halyard;

class LiquidArea;

class CharacterBuoyancy : public Node, protected NodeDebug {
	GDCLASS(CharacterBuoyancy, Node)

	bool _apply_forces = true;
	bool _ignore_waves = false;

	float _submerged_linear_drag = 3.0f; // drag when submerged should be high
	Vector3 _linear_drag_scale = Vector3(1.0f, 1.0f, 1.0f);

	Ref<BuoyancyMaterial> _buoyancy_material;
	ProbeBuoyancy _probe_buoyancy;
	float _last_submerged_ratio = 0.0f;

	uint64_t _buoyancy_time = 0; // us
	Vector3 _gravity = Vector3(0, -9.81, 0);

	void _update_last_transforms();

	// Debug
	void _update_debug_mesh() override;

protected:
	static void _bind_methods();
	void _notification(int p_what);

public:
	CharacterBuoyancy();
	~CharacterBuoyancy() override;

	PackedStringArray _get_configuration_warnings() const;

	void apply_buoyancy_velocity(float delta);

	// accessors
	void set_buoyancy_material(const Ref<BuoyancyMaterial> &p_material);
	Ref<BuoyancyMaterial> get_buoyancy_material() const;

	void set_liquid_area(LiquidArea *liquid_area);
	LiquidArea *get_liquid_area() const;

	void set_probes(const PackedVector3Array &local_probes);
	PackedVector3Array get_probes() const;

	void set_apply_forces(bool enabled);
	bool get_apply_forces() const;

	// Just use a flat plane for the liquid. Considerably faster.
	void set_ignore_waves(bool p_ignore);
	bool get_ignore_waves() const;

	void set_mass(float mass);
	float get_mass() const;

	void set_gravity(const Vector3 &gravity);
	Vector3 get_gravity() const;

	float get_submerged_ratio() const;
	uint64_t get_buoyancy_time() const;
	float get_average_depth() const;

	// Debug
	void set_show_debug(bool show);
	bool get_show_debug() const;

	void set_debug_color(const Color &color);
	Color get_debug_color() const;
};
