#include "character_buoyancy.h"
#include "liquid_area.h"
#include "halyard_utils.h"

#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/time.hpp>
#include <cmath>

using namespace godot;

CharacterBuoyancy::CharacterBuoyancy()
: NodeDebug(Object::cast_to<Node>(this)) {
	// default color
	_debug_color = Color(0.0f, 0.8f, 1.0f, 0.2f);
}

CharacterBuoyancy::~CharacterBuoyancy() {
}

PackedStringArray CharacterBuoyancy::_get_configuration_warnings() const {
	PackedStringArray what;

	CharacterBody3D *body = Object::cast_to<CharacterBody3D>(get_parent());
	if (!body && _apply_forces) {
		what.append("Buoyancy must be a child of a CharacterBody3D for forces to be applied.");
	}

	// NOTE: there are legitimate reasons to not have a liquid area, so don't nag about that.
	return what;
}

void CharacterBuoyancy::_notification(int p_what) {
    // manage debugging
    NodeDebug::_debug_notification(p_what);

	switch (p_what) {
		case NOTIFICATION_READY: {
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

		case NOTIFICATION_INTERNAL_PROCESS: {
			// in engine update this continuously so we can get visual feedback
			if (Engine::get_singleton()->is_editor_hint() == true) {
				if (_show_debug)
					set_debug_mesh_dirty();
			}
		} break;

        // velocity is applied in the internal physics process so that submerged is updated
        // for derived classes before normal physics process
		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
			if (!Engine::get_singleton()->is_editor_hint() && is_node_ready()) {
				
					uint64_t time = Time::get_singleton()->get_ticks_usec();

					// optionally apply them
					_update_last_transforms();
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
	_last_transforms.resize(_probes.size());
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
	_submerged_linear_drag = drag;
}

float CharacterBuoyancy::get_submerged_linear_drag() const {
	return _submerged_linear_drag;
}

void CharacterBuoyancy::set_linear_drag_scale(const Vector3 &scale) {
	_linear_drag_scale = scale;
}

Vector3 CharacterBuoyancy::get_linear_drag_scale() const {
	return _linear_drag_scale;
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

float CharacterBuoyancy::get_average_depth() const {
	CharacterBody3D *body = Object::cast_to<CharacterBody3D>(get_parent());
	
	if (!body) {
		return 0.0f;
	}

	const int probe_count = _probes.size();
	if (probe_count == 0 || probe_count != _last_transforms.size()) {
		return 0.0f;
	}

	const Transform3D body_transform = body->get_global_transform();
	float total_depth = 0.0f;

	for (int i = 0; i < probe_count; ++i) {
		// Get probe position in global space
		Vector3 probe = body_transform.xform(_probes[i]);
		
		// Get cached liquid surface position
		Vector3 liquid_pos = _last_transforms[i].origin;
		
		// Calculate depth (negative when underwater)
		float depth = probe.y - liquid_pos.y;
		total_depth += depth;
	}

	return total_depth / (float)probe_count;
}

// Debug
void CharacterBuoyancy::set_show_debug(bool show) {
    _show_debug = show;
    set_debug_mesh_dirty();
}

bool CharacterBuoyancy::get_show_debug() const {
    return _show_debug;
}

void CharacterBuoyancy::set_debug_color(const Color &color) {
    _debug_color = color;
    set_debug_mesh_dirty();
}

Color CharacterBuoyancy::get_debug_color() const {
    return _debug_color;
}

void CharacterBuoyancy::_update_debug_mesh() {
    if (_debug_mesh_instance == nullptr) return;
    if (_debug_mesh.is_valid() == false) return;

    PackedVector3Array vertices;
    PackedInt32Array indices;

    // clear previous surfaces
    _debug_mesh->clear_surfaces();

    // Draw probe positions
	const float size = 0.1f;
    if (_probes.size() > 0) {
        if (_node) {
            Transform3D xform;
            
            for (int idx = 0; idx < _probes.size(); idx++) {
                Vector3 point = _probes[idx];
                
                // X axis line
				vertices.append(point + Vector3(-size, 0, 0));
				vertices.append(point + Vector3(size, 0, 0));

				// Y axis line
				vertices.append(point + Vector3(0, -size, 0));
				vertices.append(point + Vector3(0, size, 0));

				// Z axis line
				vertices.append(point + Vector3(0, 0, -size));
				vertices.append(point + Vector3(0, 0, size));

				// Indices for the 3 lines (6 vertices per point)
				int base = idx * 6;
				indices.append(base + 0);
				indices.append(base + 1);

				indices.append(base + 2);
				indices.append(base + 3);

				indices.append(base + 4);
				indices.append(base + 5);
            }
        }
    }

    if (vertices.size() > 0) {
        Array arrays;
        arrays.resize(Mesh::ARRAY_MAX);
        arrays[Mesh::ARRAY_VERTEX] = vertices;
        arrays[Mesh::ARRAY_INDEX] = indices;

        _debug_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_LINES, arrays);
        _debug_mesh->surface_set_material(0, _debug_material);
    }
}

void CharacterBuoyancy::_update_last_transforms() {
    CharacterBody3D *body = Object::cast_to<CharacterBody3D>(get_parent());
    
	ERR_FAIL_NULL_MSG(body, "CharacterBuoyancy must be a child of a CharacterBody3D to update submerged state.");

	// NOTE: there are legitimate reasons to not have a liquid area or probes, so just return
    const int probe_count = _probes.size();
	_last_transforms.resize(probe_count);

    if (probe_count == 0) return;
	if (_liquid_area == nullptr) return;

	int submerged_count = 0;

    // resize cache to match probe count
    const Transform3D body_transform = body->get_global_transform();

    for (int i = 0; i < probe_count; ++i) {
        // probes are in local space, transform to global
        Vector3 probe = body_transform.xform(_probes[i]);

        // get liquid transform at this point
        Transform3D liquid_xform = _liquid_area->get_liquid_transform(probe);
        bool point_submerged = liquid_xform.origin.y > probe.y;

        // cache the transform for use in apply_buoyancy_velocity
        _last_transforms.write[i] = liquid_xform;

        if (point_submerged) {
            submerged_count += 1;
        }
    }

    // update submerged ratio and notify if changed
    float ratio = (float)submerged_count / (float)probe_count;
    
    // notify on change from completely submerged to not submerged and vice versa
    bool changed = Math::is_zero_approx(_submerged_ratio) != Math::is_zero_approx(ratio);
    _submerged_ratio = ratio;
    if (changed){
        emit_signal("submerged_changed");
    }
}

void CharacterBuoyancy::apply_buoyancy_velocity(float delta) {
    CharacterBody3D *body = Object::cast_to<CharacterBody3D>(get_parent());

    ERR_FAIL_NULL_MSG(body, "CharacterBuoyancy must be a child of a CharacterBody3D to apply buoyancy.");
    ERR_FAIL_NULL_MSG(_liquid_area, "No LiquidArea assigned to CharacterBuoyancy.");

    ERR_FAIL_COND_MSG(delta <= 0.0f, "Delta time must be positive to apply buoyancy.");
    ERR_FAIL_COND_MSG(_mass <= 0.0f, "Mass must be positive to apply buoyancy.");
    ERR_FAIL_COND_MSG(_probes.size() != _last_transforms.size(), "Probe count and cached transform count mismatch.");

    const int probe_count = _probes.size();
    if (probe_count == 0) {
        return;
    }

    // Pre-calculate constants
	const Vector3 gravity = body->get_gravity();
	const Transform3D body_transform = body->get_global_transform();
	const Basis basis = body_transform.basis.orthonormalized();
	const Vector3 one = Vector3(1, 1, 1);
	const float full_submerged_depth = halyard::calculate_full_submerged_depth(_probes, body_transform);

	// bail early if there is no depth to submerge
	if (full_submerged_depth==0.0f) {
		return;
	}

	// This represent the portion of mass each probe affects
	const float probe_ratio = 1.0f / probe_count;
	const float fluid_density = _liquid_area->get_density();

	const float character_density = fluid_density /get_buoyancy();

    // retrieve velocity
    Vector3 velocity = body->get_velocity();

	// apply buoyancy for each submerged probe
    for (int i = 0; i < probe_count; ++i) {

		// probes are in local space, transform to global
        Vector3 probe = body_transform.xform(_probes[i]);

        // use cached liquid transform from _update_last_transforms
        Transform3D wave_xform = _last_transforms[i];
		Vector3 wave_pos = wave_xform.origin;

		// Calculate wave depth as a ratio between not submerged and fully submerged
		// this will be used to calculate how much of the probes contribution is submerged
		float wave_depth = -(probe.y - wave_pos.y);
		wave_depth = Math::clamp(wave_depth / full_submerged_depth, -1.0f, 0.0f);

		if (wave_depth < 0.0f) {
            // Each probe affects 1/N of the volume.
			//  We can get the total volume from the body's mass and density.
			float probe_volume = (get_mass() / character_density) * probe_ratio * -wave_depth;

			// Calculate forces
			Vector3 wave_normal = wave_xform.basis.get_column(1);
			wave_normal = wave_normal.lerp(Vector3(0, 1, 0), probe_ratio).normalized();
			Vector3 wave_force = (wave_normal * -gravity.y) * fluid_density * probe_volume;

            // apply to velocity
            velocity += wave_force / get_mass() * delta;

            // apply linear submerged drag, in local space
			Vector3 linear_drag = one - _submerged_linear_drag * _linear_drag_scale * delta * probe_ratio;
			linear_drag = linear_drag.max(Vector3(0, 0, 0)); // prevent drag from going negative
			Vector3 local_vel = body->get_basis().xform_inv(velocity);
			velocity = body->get_basis().xform(local_vel * linear_drag);
        }
    }

    // write back velocity
    body->set_velocity(velocity);
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
	ClassDB::bind_method(D_METHOD("set_linear_drag_scale", "scale"), &CharacterBuoyancy::set_linear_drag_scale);
	ClassDB::bind_method(D_METHOD("get_linear_drag_scale"), &CharacterBuoyancy::get_linear_drag_scale);
	ClassDB::bind_method(D_METHOD("set_gravity", "gravity"), &CharacterBuoyancy::set_gravity);
	ClassDB::bind_method(D_METHOD("get_gravity"), &CharacterBuoyancy::get_gravity);
    ClassDB::bind_method(D_METHOD("set_probes", "probes"), &CharacterBuoyancy::set_probes);
    ClassDB::bind_method(D_METHOD("get_probes"), &CharacterBuoyancy::get_probes);
	ClassDB::bind_method(D_METHOD("get_buoyancy_time"), &CharacterBuoyancy::get_buoyancy_time);
	ClassDB::bind_method(D_METHOD("get_submerged_ratio"), &CharacterBuoyancy::get_submerged_ratio);
	ClassDB::bind_method(D_METHOD("get_average_depth"), &CharacterBuoyancy::get_average_depth);

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
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "linear_drag_scale"), "set_linear_drag_scale", "get_linear_drag_scale");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "gravity"), "set_gravity", "get_gravity");

    // Debug
    ADD_GROUP("Debug", "");
    ClassDB::bind_method(D_METHOD("set_show_debug", "show"), &CharacterBuoyancy::set_show_debug);
    ClassDB::bind_method(D_METHOD("get_show_debug"), &CharacterBuoyancy::get_show_debug);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "show_debug"), "set_show_debug", "get_show_debug");

    ClassDB::bind_method(D_METHOD("set_debug_color", "color"), &CharacterBuoyancy::set_debug_color);
    ClassDB::bind_method(D_METHOD("get_debug_color"), &CharacterBuoyancy::get_debug_color);
    ADD_PROPERTY(PropertyInfo(Variant::COLOR, "debug_color"), "set_debug_color", "get_debug_color");

    ADD_SIGNAL(MethodInfo("submerged_changed"));
}