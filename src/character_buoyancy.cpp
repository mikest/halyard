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

	// defaults for probe_buoyancy behavior
	_probe_buoyancy.set_buoyancy(1.5f);	// less dense as water
	_probe_buoyancy.set_mass(150.0f); // kg
}

CharacterBuoyancy::~CharacterBuoyancy() {
}

PackedStringArray CharacterBuoyancy::_get_configuration_warnings() const {
	PackedStringArray what;

	CharacterBody3D *body = Object::cast_to<CharacterBody3D>(get_parent());
	if (!body && _apply_forces) {
		what.append("Buoyancy must be a child of a CharacterBody3D for forces to be applied.");
	}

	if (!_buoyancy_material.is_valid()) {
		what.append("No buoyancy material assigned. Default values will be used.");
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

			// if no buoyancy material is assigned a default one
			if (Engine::get_singleton()->is_editor_hint() == false) {
				if (!_buoyancy_material.is_valid()) {
					_buoyancy_material.instantiate();
				}
			}
		} break;

		case NOTIFICATION_ENTER_TREE: {
			if (Engine::get_singleton()->is_editor_hint() == false) {
				if (_probe_buoyancy.get_liquid_area() == nullptr) {
					SceneTree *tree = get_tree();
					_probe_buoyancy.set_liquid_area(LiquidArea::get_liquid_area(tree));
				}
			}
		} break;

		case NOTIFICATION_EXIT_TREE: {
		 	_probe_buoyancy.set_liquid_area(nullptr);
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
			if (!Engine::get_singleton()->is_editor_hint() && \
				is_node_ready() && \
				_probe_buoyancy.get_liquid_area() != nullptr) {
				
					uint64_t time = Time::get_singleton()->get_ticks_usec();

					// optionally apply them
					_update_last_transforms();
					if (_apply_forces){
						float delta = get_physics_process_delta_time();
						apply_buoyancy_velocity(delta);
					}

					// Check if submerged changed and emit signal. We only track entering/exiting water as the ratio can change on every frame.
					float current_ratio = get_submerged_ratio();
					if (Math::is_zero_approx(current_ratio) != Math::is_zero_approx(_last_submerged_ratio)) {
						emit_signal("submerged_changed");
					}
					_last_submerged_ratio = current_ratio;

					uint64_t elapsed = Time::get_singleton()->get_ticks_usec() - time;
					_buoyancy_time = elapsed;
			}
		} break;
	}
}

// Property getters/setters
void CharacterBuoyancy::set_liquid_area(LiquidArea *p_area) {
	_probe_buoyancy.set_liquid_area(p_area);

    // notify editor of change
	if (Engine::get_singleton()->is_editor_hint()){
		update_configuration_warnings();
	}
}


LiquidArea* CharacterBuoyancy::get_liquid_area() const {
	return _probe_buoyancy.get_liquid_area();
}

void CharacterBuoyancy::set_buoyancy_material(const Ref<BuoyancyMaterial> &p_material) {
	_buoyancy_material = p_material;
}

Ref<BuoyancyMaterial> CharacterBuoyancy::get_buoyancy_material() const {
	return _buoyancy_material;
}


void CharacterBuoyancy::set_probes(const PackedVector3Array &local_probes) {
	_probe_buoyancy.set_probes(local_probes);
	set_debug_mesh_dirty();
}

PackedVector3Array CharacterBuoyancy::get_probes() const {
    return _probe_buoyancy.get_probes();
}

// accessors
void CharacterBuoyancy::set_mass(float mass) {
	_probe_buoyancy.set_mass(mass);
}

float CharacterBuoyancy::get_mass() const {
	return _probe_buoyancy.get_mass();
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

void CharacterBuoyancy::set_ignore_waves(bool p_ignore) {
	_ignore_waves = p_ignore;
	_probe_buoyancy.set_ignore_waves(p_ignore);
}

bool CharacterBuoyancy::get_ignore_waves() const {
	return _ignore_waves;
}

uint64_t CharacterBuoyancy::get_buoyancy_time() const {
	return _buoyancy_time;
}

float CharacterBuoyancy::get_submerged_ratio() const {
	return _probe_buoyancy.get_submerged_ratio();
}

float CharacterBuoyancy::get_average_depth() const {
	CharacterBody3D *body = Object::cast_to<CharacterBody3D>(get_parent());
	
	if (!body) {
		return 0.0f;
	}

	auto probes = _probe_buoyancy.get_probes();
	auto last_transforms = _probe_buoyancy.get_last_transforms();

	const int probe_count = probes.size();
	if (probe_count == 0 || probe_count != last_transforms.size()) {
		return 0.0f;
	}

	const Transform3D body_transform = body->get_global_transform();
	float total_depth = 0.0f;

	for (int i = 0; i < probe_count; ++i) {
		// Get probe position in global space
		Vector3 probe = body_transform.xform(probes[i]);
		
		// Get cached liquid surface position
		Vector3 liquid_pos = last_transforms[i].origin;
		
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
	auto probes = _probe_buoyancy.get_probes();
	const float size = 0.1f;
    if (probes.size() > 0) {
        if (_node) {
            Transform3D xform;
            
            for (int idx = 0; idx < probes.size(); idx++) {
                Vector3 point = probes[idx];
                
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

	_probe_buoyancy.update_transforms(body->get_global_transform());
}


void CharacterBuoyancy::apply_buoyancy_velocity(float delta) {
	CharacterBody3D *body = Object::cast_to<CharacterBody3D>(get_parent());

	ERR_FAIL_NULL_MSG(body, "CharacterBuoyancy must be a child of a CharacterBody3D to apply buoyancy.");
	ERR_FAIL_COND_MSG(delta <= 0.0f, "Delta time must be positive to apply buoyancy.");
	ERR_FAIL_COND_MSG(!_buoyancy_material.is_valid(), "Buoyancy material must be valid to apply mesh buoyancy forces.");

	const float mass = _probe_buoyancy.get_mass();
	ERR_FAIL_COND_MSG(mass <= 0.0f, "Mass must be positive to apply buoyancy.");

	// update forces
	_probe_buoyancy.set_buoyancy(_buoyancy_material->get_buoyancy());
	_probe_buoyancy.update_forces(body->get_global_transform(), _gravity);

	const auto probes = _probe_buoyancy.get_probes();
	const auto forces = _probe_buoyancy.get_forces();
	ERR_FAIL_COND_MSG(probes.size() != forces.size(), "Probe and forces count mismatch.");

	// constants
	const Vector3 one = Vector3(1, 1, 1);
	const float probe_ratio = 1.0f / probes.size();

	Vector3 submerged_linear_drag =
		_buoyancy_material->get_linear_drag() *
		_buoyancy_material->get_linear_drag_scale();

	// retrieve velocity
    Vector3 velocity = body->get_velocity();

	// add velocity changes from forces
	const int probe_count = probes.size();
	for (int i = 0; i < probe_count; ++i) {

		// don't apply drag if there is no force
		const Vector3 force = forces[i];
		if (force.length()) {
			velocity += force / mass * delta;

			// apply linear submerged drag, in local space
			Vector3 linear_drag = one - submerged_linear_drag * delta * probe_ratio;
			linear_drag = linear_drag.max(Vector3(0, 0, 0)); // prevent drag from going negative
			Vector3 local_vel = body->get_basis().xform_inv(velocity);
			velocity = body->get_basis().xform(local_vel * linear_drag);
		}
	}

	// write back velocity
	body->set_velocity(velocity);
}


void CharacterBuoyancy::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_apply_forces", "enabled"), &CharacterBuoyancy::set_apply_forces);
	ClassDB::bind_method(D_METHOD("get_apply_forces"), &CharacterBuoyancy::get_apply_forces);
	ClassDB::bind_method(D_METHOD("set_mass", "mass"), &CharacterBuoyancy::set_mass);
	ClassDB::bind_method(D_METHOD("get_mass"), &CharacterBuoyancy::get_mass);
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

	ClassDB::bind_method(D_METHOD("set_buoyancy_material", "buoyancy_material"), &CharacterBuoyancy::set_buoyancy_material);
	ClassDB::bind_method(D_METHOD("get_buoyancy_material"), &CharacterBuoyancy::get_buoyancy_material);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "buoyancy_material", PROPERTY_HINT_RESOURCE_TYPE, "BuoyancyMaterial"), "set_buoyancy_material", "get_buoyancy_material");

    ADD_PROPERTY(PropertyInfo(Variant::PACKED_VECTOR3_ARRAY, "probes"), "set_probes", "get_probes");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "apply_forces"), "set_apply_forces", "get_apply_forces");

	ClassDB::bind_method(D_METHOD("set_ignore_waves", "ignore_waves"), &CharacterBuoyancy::set_ignore_waves);
	ClassDB::bind_method(D_METHOD("get_ignore_waves"), &CharacterBuoyancy::get_ignore_waves);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "ignore_waves"), "set_ignore_waves", "get_ignore_waves");
	
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "mass", PROPERTY_HINT_NONE, "kg"), "set_mass", "get_mass");
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