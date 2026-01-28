#include "character_buoyancy.h"
#include "liquid_area.h"

#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/time.hpp>
#include <cmath>

using namespace godot;

CharacterBuoyancy::CharacterBuoyancy() {
}

CharacterBuoyancy::~CharacterBuoyancy() {
}

PackedStringArray CharacterBuoyancy::_get_configuration_warnings() const {
	PackedStringArray what;

	CharacterBody3D *body = Object::cast_to<CharacterBody3D>(get_parent());
	if (!body && _apply_forces) {
		what.append("Buoyancy must be a child of a CharacterBody3D for forces to be applied.");
	}

	// if (!_liquid_area) {
	// 	what.append("Missing LiquidArea. First LiquidArea in the scene tree will be used,\n or liquid level will be Vector3.ZERO");
	// }

	return what;
}

void CharacterBuoyancy::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_READY: {
			if (Engine::get_singleton()->is_editor_hint()) {
			}
			set_process_internal(true);
			set_physics_process_internal(true);
		} break;

		case NOTIFICATION_ENTER_TREE: {
			if (Engine::get_singleton()->is_editor_hint() == false) {
				if (_liquid_area == nullptr) {
					SceneTree *tree = get_tree();
					_liquid_area = LiquidArea::get_liquid_area(tree);
				}
			}
		} break;

		case NOTIFICATION_EXIT_TREE: {
		 	_liquid_area = nullptr;
		} break;

        // velocity is applied in the internal physics process so that submerged is updated
        // for derived classes before normal physics process
		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
			if (!Engine::get_singleton()->is_editor_hint() && is_node_ready()) {
				
					uint64_t time = Time::get_singleton()->get_ticks_usec();

					// optionally apply them
					if (_apply_forces){
						float delta = get_physics_process_delta_time();
						apply_buoyancy_velocity(delta);
					}

					uint64_t elapsed = Time::get_singleton()->get_ticks_usec() - time;
					_buoyancy_time = elapsed;
			}
		} break;
	}
}

// Property getters/setters
void CharacterBuoyancy::set_liquid_area(LiquidArea *p_area) {
	_liquid_area = p_area;

    // notify editor of change
	if (Engine::get_singleton()->is_editor_hint()){
		update_configuration_warnings();
	}
}


LiquidArea* CharacterBuoyancy::get_liquid_area() const {
	return _liquid_area;
}


void CharacterBuoyancy::set_probes(const PackedVector3Array &local_probes) {
    _probes = local_probes;
}

PackedVector3Array CharacterBuoyancy::get_probes() const {
    return _probes;
}

// accessors
void CharacterBuoyancy::set_buoyancy(float buoyancy) {
	_buoyancy = buoyancy;
}

float CharacterBuoyancy::get_buoyancy() const {
	return _buoyancy;
}

void CharacterBuoyancy::set_mass(float mass) {
	_mass = mass;
}

float CharacterBuoyancy::get_mass() const {
	return _mass;
}

void CharacterBuoyancy::set_wave_influence(float influence) {
	_wave_influence = influence;
}

float CharacterBuoyancy::get_wave_influence() const {
	return _wave_influence;
}

void CharacterBuoyancy::set_submerged_linear_drag(float drag) {
	_submerged_drag_linear = drag;
}

float CharacterBuoyancy::get_submerged_linear_drag() const {
	return _submerged_drag_linear;
}

void CharacterBuoyancy::set_gravity(const Vector3 &gravity) {
	_gravity = gravity;
}

Vector3 CharacterBuoyancy::get_gravity() const {
	return _gravity;
}

void CharacterBuoyancy::set_apply_forces(bool enabled) {
	_apply_forces = enabled;
}

bool CharacterBuoyancy::get_apply_forces() const {
	return _apply_forces;
}

uint64_t CharacterBuoyancy::get_buoyancy_time() const {
	return _buoyancy_time;
}

float CharacterBuoyancy::get_submerged_ratio() const {
	return _submerged_ratio;
}

void CharacterBuoyancy::apply_buoyancy_velocity(float delta) {
    CharacterBody3D *body = Object::cast_to<CharacterBody3D>(get_parent());
    
    ERR_FAIL_NULL_MSG(body, "CharacterBuoyancy must be a child of a CharacterBody3D to apply buoyancy.");
    ERR_FAIL_NULL_MSG(_liquid_area, "No LiquidArea assigned to CharacterBuoyancy.");

    ERR_FAIL_COND_MSG(delta <= 0.0f, "Delta time must be positive to apply buoyancy.");
    ERR_FAIL_COND_MSG(_mass <= 0.0f, "Mass must be positive to apply buoyancy.");

	int submerged_count = 0;

    // retrieve velocity
    Vector3 velocity = body->get_velocity();
    for (int i = 0; i < _probes.size(); ++i) {
        Vector3 probe = _probes[i];

        // probes are in local space
        probe = body->get_global_transform().xform(probe);

        // liquid is in world space
        Transform3D liquid_xform;
        bool point_submerged = false;
        if (_liquid_area) {
            liquid_xform = _liquid_area->get_liquid_transform(probe);
            point_submerged = liquid_xform.origin.y > probe.y;
        }

        // add this probes contribution
        float probe_proportion = _buoyancy * 1.0 / (float)_probes.size();
		if (point_submerged) {
			// just one is enough to mark the body as submerged
			submerged_count += 1;

            // invert gravity to get buoyancy. depth is negative, so will invert gravity
            float depth = probe.y - liquid_xform.origin.y;
            Vector3 force = liquid_xform.basis.xform(_gravity * _mass * depth); 

            // apply to velocity
            velocity += (force / _mass) * delta * probe_proportion;

            // apply linear submerged drag
            float damping = 1.0 - (_submerged_drag_linear * probe_proportion * delta);
            damping = Math::max(damping, 0.0f);

            velocity *= damping;
        }
    }

    // write back velocity
    body->set_velocity(velocity);

	// update submerged ratio and notify if changed
	float ratio = 0.0f;
	if (_probes.size() > 0) {
		ratio = (float)submerged_count / (float)_probes.size();
	}
	
    // notify on change from completely submerged to not submerged and vice versa
	bool changed = Math::is_zero_approx(_submerged_ratio) != Math::is_zero_approx(ratio);
	_submerged_ratio = ratio;
	if (changed){
		emit_signal("submerged_changed");
	}
}


void CharacterBuoyancy::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_buoyancy", "buoyancy"), &CharacterBuoyancy::set_buoyancy);
	ClassDB::bind_method(D_METHOD("get_buoyancy"), &CharacterBuoyancy::get_buoyancy);
	ClassDB::bind_method(D_METHOD("set_apply_forces", "enabled"), &CharacterBuoyancy::set_apply_forces);
	ClassDB::bind_method(D_METHOD("get_apply_forces"), &CharacterBuoyancy::get_apply_forces);
	ClassDB::bind_method(D_METHOD("set_mass", "mass"), &CharacterBuoyancy::set_mass);
	ClassDB::bind_method(D_METHOD("get_mass"), &CharacterBuoyancy::get_mass);
	ClassDB::bind_method(D_METHOD("set_wave_influence", "influence"), &CharacterBuoyancy::set_wave_influence);
	ClassDB::bind_method(D_METHOD("get_wave_influence"), &CharacterBuoyancy::get_wave_influence);
	ClassDB::bind_method(D_METHOD("set_submerged_linear_drag", "drag"), &CharacterBuoyancy::set_submerged_linear_drag);
	ClassDB::bind_method(D_METHOD("get_submerged_linear_drag"), &CharacterBuoyancy::get_submerged_linear_drag);
	ClassDB::bind_method(D_METHOD("set_gravity", "gravity"), &CharacterBuoyancy::set_gravity);
	ClassDB::bind_method(D_METHOD("get_gravity"), &CharacterBuoyancy::get_gravity);
    ClassDB::bind_method(D_METHOD("set_probes", "probes"), &CharacterBuoyancy::set_probes);
    ClassDB::bind_method(D_METHOD("get_probes"), &CharacterBuoyancy::get_probes);
	ClassDB::bind_method(D_METHOD("get_buoyancy_time"), &CharacterBuoyancy::get_buoyancy_time);
	ClassDB::bind_method(D_METHOD("get_submerged_ratio"), &CharacterBuoyancy::get_submerged_ratio);

    ClassDB::bind_method(D_METHOD("apply_buoyancy_velocity", "delta"), &CharacterBuoyancy::apply_buoyancy_velocity);

    ClassDB::bind_method(D_METHOD("set_liquid_area", "liquid_area"), &CharacterBuoyancy::set_liquid_area);
	ClassDB::bind_method(D_METHOD("get_liquid_area"), &CharacterBuoyancy::get_liquid_area);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "liquid_area", PROPERTY_HINT_NODE_TYPE, "LiquidArea"), "set_liquid_area", "get_liquid_area");
    ADD_PROPERTY(PropertyInfo(Variant::PACKED_VECTOR3_ARRAY, "probes"), "set_probes", "get_probes");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "apply_forces"), "set_apply_forces", "get_apply_forces");
	
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "mass", PROPERTY_HINT_NONE, "kg"), "set_mass", "get_mass");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "buoyancy", PROPERTY_HINT_RANGE, "0,100,0.1"), "set_buoyancy", "get_buoyancy");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "wave_influence", PROPERTY_HINT_RANGE, "0,1,0.1"), "set_wave_influence", "get_wave_influence");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "submerged_linear_drag", PROPERTY_HINT_RANGE, "0,10,0.1"), "set_submerged_linear_drag", "get_submerged_linear_drag");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "gravity"), "set_gravity", "get_gravity");

    ADD_SIGNAL(MethodInfo("submerged_changed"));
}