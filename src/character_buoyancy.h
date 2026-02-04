#pragma once

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/core/binder_common.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/variant/basis.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/classes/character_body3d.hpp>

#include "node_debug.h"

using namespace godot;

class LiquidArea;

class CharacterBuoyancy : public Node, protected NodeDebug {
    GDCLASS(CharacterBuoyancy, Node)

    LiquidArea* _liquid_area = nullptr;

    bool _apply_forces = true;
    
    float _mass = 150.0f;   // kg
    float _buoyancy = 1.0f; // mass multiplier
    float _wave_influence = 1.0f;
    float _submerged_drag_linear = 3.0f;    // drag when submerged should be high

    PackedVector3Array _probes;
    float _full_submerged_depth = 0.0f;
    Vector<Transform3D> _last_transforms;
    uint64_t _buoyancy_time = 0; // us
    Vector3 _gravity = Vector3(0, -9.81, 0);
    float _submerged_ratio = 0.0f;

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
    void set_liquid_area(LiquidArea *liquid_area);
    LiquidArea* get_liquid_area() const;

    void set_probes(const PackedVector3Array &local_probes);
    PackedVector3Array get_probes() const;

    void set_apply_forces(bool enabled);
    bool get_apply_forces() const;

    void set_buoyancy(float buoyancy);
    float get_buoyancy() const;

    void set_mass(float mass);
    float get_mass() const;

    void set_wave_influence(float influence);
    float get_wave_influence() const;

    void set_submerged_linear_drag(float drag);
    float get_submerged_linear_drag() const;

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
