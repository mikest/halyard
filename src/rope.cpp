#include "rope.h"
#include "liquid_area.h"
#include "rope_anchor.h"
#include "rope_appearance.h"
#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/editor_interface.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/classes/physics_direct_space_state3d.hpp>
#include <godot_cpp/classes/physics_ray_query_parameters3d.hpp>
#include <godot_cpp/classes/physics_server3d.hpp>
#include <godot_cpp/classes/project_settings.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/sphere_shape3d.hpp>
#include <godot_cpp/classes/sub_viewport.hpp>
#include <godot_cpp/classes/viewport.hpp>
#include <godot_cpp/classes/world3d.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include "dd3d_cpp_api.hpp"
#include "halyard_utils.h"

#define BEHAVIOR_HINT "Free:-1,Anchored:0,Towing:1,Guided:2,Sliding:3"
#define FROM_HINT "Start:0,End:1"
#define DISTRIBUTION_HINT "Absolute:0,Relative:1,Uniform:2,Scalar:3,Real:4"

// dump initial conditions for debugging stretch grow directions.
#define DEBUG_INITIAL_POS false

// collision_layer support doesn't work yet
#define ENABLE_COLLISION_LAYER false

// Basis indexes
const int X = 0;
const int Y = 1;
const int Z = 2;

Rope::Rope() {
	_rope_mesh.instantiate();
	set_base(_rope_mesh->get_rid());

	// Initialize cached objects for reuse
	_ray_cast.instantiate();
	_ray_cast->set_exclude(_exclusion_list);
	_ray_cast->set_collide_with_areas(false);
	_ray_cast->set_collide_with_bodies(true);

	_collision_shape.instantiate();

	_shape_cast.instantiate();
	_shape_cast->set_shape(_collision_shape);
	_shape_cast->set_exclude(_exclusion_list);
	_shape_cast->set_collide_with_areas(false);
	_shape_cast->set_collide_with_bodies(true);

	// disable scaling, we should never be scaled
	set_disable_scale(true);
}

Rope::Rope(const Rope &other) {
}

Rope::~Rope() {
	if (_appearance != nullptr)
		_appearance->disconnect("changed", Callable(this, "_on_appearance_changed"));

	_clear_links();

	if (_link_shape.is_valid()) {
		PhysicsServer3D::get_singleton()->free_rid(_link_shape);
	}
}

#if 1 // Array binding is not included in GDExtension, so replicate the calls here.
#define ADD_ARRAY_COUNT(m_label, m_count_property, m_count_property_setter, m_count_property_getter, m_prefix) ClassDB_add_property_array_count(get_class_static(), m_label, m_count_property, StringName(m_count_property_setter), StringName(m_count_property_getter), m_prefix)

inline void ClassDB_add_property_array_count(const StringName &p_class, const String &p_label, const StringName &p_count_property, const StringName &p_count_setter, const StringName &p_count_getter, const String &p_array_element_prefix, uint32_t p_count_usage = PROPERTY_USAGE_DEFAULT) {
	ClassDB::add_property(p_class, PropertyInfo(Variant::INT, p_count_property, PROPERTY_HINT_NONE, "", p_count_usage | PROPERTY_USAGE_ARRAY, vformat("%s,%s", p_label, p_array_element_prefix)), p_count_setter, p_count_getter);
}
#endif

const char ANCHORS_KEY[] = "anchors/";
const char OFFSET_KEY[] = "offset";
const char FROM_KEY[] = "from";
const char NODE_PATH_KEY[] = "node_path";
const char BEHAVIOR_KEY[] = "behavior";
const char FRICTION_KEY[] = "friction";
const char POSITION_KEY[] = "position";

void Rope::_bind_methods() {
	BIND_ENUM_CONSTANT(Start);
	BIND_ENUM_CONSTANT(End);

	BIND_ENUM_CONSTANT(ABSOLUTE);
	BIND_ENUM_CONSTANT(RELATIVE);
	BIND_ENUM_CONSTANT(UNIFORM);
	BIND_ENUM_CONSTANT(SCALAR);
	BIND_ENUM_CONSTANT(REAL);

	ClassDB::bind_method(D_METHOD("get_baked_mesh"), &Rope::get_baked_mesh);
	ClassDB::bind_method(D_METHOD("get_current_rope_length"), &Rope::get_current_rope_length);
	ClassDB::bind_method(D_METHOD("get_particle_count_for_length"), &Rope::get_particle_count_for_length);

	ClassDB::bind_method(D_METHOD("get_rope_frame_count"), &Rope::get_rope_frame_count);
	ClassDB::bind_method(D_METHOD("get_rope_frame", "index"), &Rope::get_rope_frame);
	ClassDB::bind_method(D_METHOD("get_all_rope_frames"), &Rope::get_all_rope_frames);

	ClassDB::bind_method(D_METHOD("get_particle_positions"), &Rope::get_particle_positions);

	ClassDB::bind_method(D_METHOD("get_link_count"), &Rope::get_rope_frame_count);
	ClassDB::bind_method(D_METHOD("get_link", "index"), &Rope::get_rope_frame);
	ClassDB::bind_method(D_METHOD("get_all_links"), &Rope::get_all_rope_frames);

	ClassDB::bind_method(D_METHOD("get_rope_sides"), &Rope::get_rope_sides);
	ClassDB::bind_method(D_METHOD("get_rope_twist"), &Rope::get_rope_twist);
	ClassDB::bind_method(D_METHOD("get_rope_lod"), &Rope::get_rope_lod);
	ClassDB::bind_method(D_METHOD("get_start_offset"), &Rope::get_start_offset);
	ClassDB::bind_method(D_METHOD("get_end_offset"), &Rope::get_end_offset);

	ClassDB::bind_method(D_METHOD("_on_appearance_changed"), &Rope::_on_appearance_changed);

	// internal
	ClassDB::bind_method(D_METHOD("_anchor_for_particle", "particle_idx"), &Rope::_anchor_for_particle);
	ClassDB::bind_method(D_METHOD("_particle_for_anchor", "anchor_idx"), &Rope::_particle_for_anchor);
	ClassDB::bind_method(D_METHOD("_particle_stretch", "particle_idx"), &Rope::_particle_stretch);
	ClassDB::bind_method(D_METHOD("_rebuild_rope"), &Rope::_rebuild_rope);

	ClassDB::bind_method(D_METHOD("_set_initial_pos", "initial_pos"), &Rope::_set_initial_pos);
	ClassDB::bind_method(D_METHOD("_get_initial_pos"), &Rope::_get_initial_pos);
	ClassDB::bind_method(D_METHOD("_bake_initial_pos"), &Rope::_bake_initial_pos);

	// anchors
	ClassDB::bind_method(D_METHOD("set_anchor_count", "count"), &Rope::set_anchor_count);
	ClassDB::bind_method(D_METHOD("get_anchor_count"), &Rope::get_anchor_count);
	ClassDB::bind_method(D_METHOD("clear_anchors"), &Rope::clear_anchors);
	ClassDB::bind_method(D_METHOD("set_anchor_offset", "idx", "position"), &Rope::set_anchor_offset);
	ClassDB::bind_method(D_METHOD("get_anchor_offset", "idx"), &Rope::get_anchor_offset);
	ClassDB::bind_method(D_METHOD("set_anchor_from", "idx", "from"), &Rope::set_anchor_from);
	ClassDB::bind_method(D_METHOD("get_anchor_from", "idx"), &Rope::get_anchor_from);
	ClassDB::bind_method(D_METHOD("set_anchor_nodepath", "idx", "path"), &Rope::set_anchor_nodepath);
	ClassDB::bind_method(D_METHOD("get_anchor_nodepath", "idx"), &Rope::get_anchor_nodepath);
	ClassDB::bind_method(D_METHOD("set_anchor_behavior", "idx", "behavior"), &Rope::set_anchor_behavior);
	ClassDB::bind_method(D_METHOD("get_anchor_behavior", "idx"), &Rope::get_anchor_behavior);
	ClassDB::bind_method(D_METHOD("set_anchor_friction", "idx", "friction"), &Rope::set_anchor_friction);
	ClassDB::bind_method(D_METHOD("get_anchor_friction", "idx"), &Rope::get_anchor_friction);
	ClassDB::bind_method(D_METHOD("set_anchor_transform", "idx", "transform"), &Rope::set_anchor_transform);
	ClassDB::bind_method(D_METHOD("get_anchor_transform", "idx"), &Rope::get_anchor_transform);
	ClassDB::bind_method(D_METHOD("get_anchor_abs_offset", "idx"), &Rope::get_anchor_abs_offset);
	ClassDB::bind_method(D_METHOD("get_anchor_particle_count", "anchor_idx"), &Rope::get_anchor_particle_count);
	ClassDB::bind_method(D_METHOD("get_anchor_particle_position", "anchor_idx", "particle_idx"), &Rope::get_anchor_particle_position);

	ClassDB::bind_method(D_METHOD("set_appearance", "appearance"), &Rope::set_appearance);
	ClassDB::bind_method(D_METHOD("get_appearance"), &Rope::get_appearance);

	// simulation
#if ENABLE_COLLISION_LAYER
	ClassDB::bind_method(D_METHOD("set_collision_layer", "collision_layer"), &Rope::set_collision_layer);
	ClassDB::bind_method(D_METHOD("get_collision_layer"), &Rope::get_collision_layer);
#endif

	ClassDB::bind_method(D_METHOD("set_collision_mask", "collision_mask"), &Rope::set_collision_mask);
	ClassDB::bind_method(D_METHOD("get_collision_mask"), &Rope::get_collision_mask);

	ClassDB::bind_method(D_METHOD("set_liquid_area", "liquid_area"), &Rope::set_liquid_area);
	ClassDB::bind_method(D_METHOD("get_liquid_area"), &Rope::get_liquid_area);

	ClassDB::bind_method(D_METHOD("set_wind_noise", "wind_noise"), &Rope::set_wind_noise);
	ClassDB::bind_method(D_METHOD("get_wind_noise"), &Rope::get_wind_noise);

	// Anchor virtuals
	ClassDB::bind_method(D_METHOD("_notify_anchors_changed"), &Rope::_notify_anchors_changed);
	GDVIRTUAL_BIND(_update_anchors)

	ClassDB::bind_method(D_METHOD("_set_attachments_from_children"), &Rope::_set_attachments_from_children);

	// Attachment virtuals
	ClassDB::bind_method(D_METHOD("_notify_attachments_changed"), &Rope::_notify_attachments_changed);
	GDVIRTUAL_BIND(_update_attachments)
	ClassDB::bind_method(D_METHOD("_get_attachment_local_transform", "attach_idx"),
			&Rope::_get_attachment_local_transform);
	GDVIRTUAL_BIND(_get_attachment_local_transform, "attach_idx")

	// Main Properties
	EXPORT_PROPERTY(Variant::FLOAT, rope_length, Rope);
	EXPORT_PROPERTY(Variant::FLOAT, rope_width, Rope);
	EXPORT_PROPERTY_RANGED(Variant::FLOAT, particles_per_meter, Rope, "0.1,20,,hide_slider");

	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "appearance", PROPERTY_HINT_RESOURCE_TYPE, "RopeAppearance"), "set_appearance", "get_appearance");

	// Anchor array
	// ADD_GROUP("Rigging", "");
	EXPORT_PROPERTY_ENUM(grow_from, FROM_HINT, Rope);
	EXPORT_PROPERTY_ENUM(anchor_distribution, DISTRIBUTION_HINT, Rope);

	// ADD_GROUP("", "");
	ADD_ARRAY_COUNT("Anchors", "anchor_count", "set_anchor_count", "get_anchor_count", ANCHORS_KEY);

	// simulation parameters
	ADD_GROUP("Simulation", "");
	EXPORT_PROPERTY(Variant::BOOL, simulate, Rope);
	EXPORT_PROPERTY(Variant::BOOL, jitter_initial_position, Rope);
	EXPORT_PROPERTY(Variant::FLOAT, preprocess_time, Rope);
	EXPORT_PROPERTY(Variant::FLOAT, simulation_rate, Rope);
	EXPORT_PROPERTY_RANGED(Variant::INT, stiffness_iterations, Rope, "1,50,1,hide_slider");
	EXPORT_PROPERTY_RANGED(Variant::FLOAT, stiffness, Rope, "0.01,1.99,0.01");
	EXPORT_PROPERTY_RANGED(Variant::FLOAT, friction, Rope, "0.0,1.0,0.01");
	EXPORT_PROPERTY_RANGED(Variant::FLOAT, tension_force_scale, Rope, "0.0,10.0,0.1");
	EXPORT_PROPERTY(Variant::FLOAT, max_tension_force, Rope);

#if ENABLE_COLLISION_LAYER
	ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_layer", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_layer", "get_collision_layer");
#endif
	ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_mask", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_mask", "get_collision_mask");

	EXPORT_PROPERTY_RANGED(Variant::FLOAT, collision_margin, Rope, "0.0,0.1,0.001");
	EXPORT_PROPERTY(Variant::BOOL, collision_use_shape_cast, Rope);

	// Buoyancy
	ADD_GROUP("Buoyancy", "");
	EXPORT_PROPERTY(Variant::BOOL, apply_buoyancy, Rope);
	EXPORT_PROPERTY(Variant::FLOAT, buoyancy_scale, Rope);
	EXPORT_PROPERTY(Variant::FLOAT, submerged_drag, Rope);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "liquid_area", PROPERTY_HINT_NODE_TYPE, "LiquidArea"), "set_liquid_area", "get_liquid_area");

	// Wind
	ADD_GROUP("Wind", "");
	EXPORT_PROPERTY(Variant::BOOL, apply_wind, Rope);
	EXPORT_PROPERTY(Variant::FLOAT, wind_scale, Rope);
	EXPORT_PROPERTY(Variant::VECTOR3, wind, Rope);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "wind_noise", PROPERTY_HINT_RESOURCE_TYPE, "FastNoiseLite"), "set_wind_noise", "get_wind_noise");

	// Gravity
	ADD_GROUP("Gravity", "");
	EXPORT_PROPERTY(Variant::BOOL, apply_gravity, Rope);
	EXPORT_PROPERTY(Variant::VECTOR3, gravity, Rope);
	EXPORT_PROPERTY(Variant::FLOAT, gravity_scale, Rope);

	// Damping
	ADD_GROUP("Damping", "");
	EXPORT_PROPERTY(Variant::BOOL, apply_damping, Rope);
	EXPORT_PROPERTY(Variant::FLOAT, damping_factor, Rope);

	// Debug
	ADD_GROUP("Debug", "");
	EXPORT_PROPERTY(Variant::BOOL, debug, Rope);
	EXPORT_PROPERTY(Variant::COLOR, debug_color, Rope);
	EXPORT_PROPERTY(Variant::BOOL, debug_collision, Rope);

	// Storage-only property hidden from the editor, persists initial particle positions across scene saves.
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_VECTOR3_ARRAY, "initial_pos", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_STORAGE | PROPERTY_USAGE_INTERNAL), "_set_initial_pos", "_get_initial_pos");
}

#pragma region Runtime

void Rope::_notification(int p_what) {
	double delta = Engine::get_singleton()->is_in_physics_frame() ? get_physics_process_delta_time() : get_process_delta_time();
	bool visible = is_visible_in_tree();

	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {
			if (Engine::get_singleton()->is_editor_hint() == false) {
				if (_liquid_area_id == 0) {
					SceneTree *tree = get_tree();
					auto *found = LiquidArea::get_liquid_area(tree);
					_liquid_area_id = found ? found->get_instance_id() : 0;
				}
			}
			notify_property_list_changed();
		} break;

		case NOTIFICATION_ENTER_WORLD: {
			_internal_enter_world();
		} break;

		case NOTIFICATION_EXIT_WORLD: {
			_internal_exit_world();
		} break;

		case NOTIFICATION_TRANSFORM_CHANGED: {
		} break;

		case NOTIFICATION_VISIBILITY_CHANGED: {
			_set_links_visible(visible);
		} break;

		case NOTIFICATION_READY: {
			_internal_ready();
		} break;

		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
			_internal_physics_process(delta);
		} break;

		case NOTIFICATION_INTERNAL_PROCESS: {
			_internal_process(delta);
		} break;

		case NOTIFICATION_EXIT_TREE: {
			_liquid_area_id = 0;
		} break;
	}
}

void Rope::_internal_enter_world() {
	Ref<World3D> w3d = get_world_3d();
	auto ps = PhysicsServer3D::get_singleton();
	auto rs = RenderingServer::get_singleton();
	ERR_FAIL_NULL(w3d);

	if (w3d.is_valid() && rs && ps) {
		RID space = w3d->get_space();
		RID scenario = w3d->get_scenario();
		bool visible = is_visible_in_tree();

		for (auto link : _links) {
			if (link.physics_body.is_valid()) {
				ps->body_set_space(link.physics_body, space);
				ps->body_set_state(link.physics_body, PhysicsServer3D::BODY_STATE_TRANSFORM, link.xform);
			}
			if (link.mesh_instance.is_valid()) {
				rs->instance_set_scenario(link.mesh_instance, scenario);
				rs->instance_set_visible(link.mesh_instance, visible);
				rs->instance_set_transform(link.mesh_instance, link.xform);
			}
		}
	}
}

void Rope::_internal_exit_world() {
	auto ps = PhysicsServer3D::get_singleton();
	auto rs = RenderingServer::get_singleton();
	bool visible = is_visible_in_tree();

	for (auto link : _links) {
		if (link.physics_body.is_valid()) {
			ps->body_set_space(link.physics_body, RID());
		}
		if (link.mesh_instance.is_valid()) {
			rs->instance_set_scenario(link.mesh_instance, RID());
			rs->instance_set_visible(link.mesh_instance, visible);
			// rs->instance_attach_skeleton(instance, RID());
		}
	}
}

void Rope::_internal_ready(void) {
	set_process_internal(true);
	set_physics_process_internal(true);
	_queue_redraw();

	// if _anchors is empty, add self as start
	if (_anchors.size() == 0) {
		_anchors.push_back(Anchor(0.0, get_global_transform(), AnchorBehavior::ANCHORED, nullptr, this));
	}
	notify_property_list_changed();
}

void Rope::_internal_process(double delta) {
	if (!is_inside_tree()) {
		return;
	}

	if (_pop_is_dirty()) {
		// rebuild if necessary
		if (_attachments_dirty) {
			_update_attachments();
			_attachments_dirty = false;
		}

		// cancel out the rotation for our transform.
		// the simulation can't work with it.
		set_global_transform(Transform3D(Basis(), get_global_position()));

		_draw_rope();
		_debug_draw_rope();
	}
}

void Rope::_internal_physics_process(double delta) {
	SCOPED_TIMER(Rope_internal_physics_process);
	if (!is_inside_tree()) {
		return;
	}

	_time += delta;
	_simulation_delta += delta;

	auto simulation_step = 1.0 / float(_simulation_rate);
	if (_simulation_delta < simulation_step)
		return;

	if (_anchors_dirty) {
		_update_anchors();
		_rebuild_anchors();

		// _queue_rope_rebuild();
		_anchors_dirty = false;
	}

	if (_pop_rebuild()) {
		_rebuild_rope();
	}

	if (_particles.size() >= 2) {
		_update_physics(float(simulation_step), _simulate ? 1 : 0);
		if (_simulate)
			_queue_redraw();
	}

	// reset delta when we simulate
	_simulation_delta = 0.0;
}

#pragma endregion

#pragma region Instancing

void Rope::_set_links_visible(bool p_visible) {
	auto rs = RenderingServer::get_singleton();

	for (auto &link : _links) {
		if (link.mesh_instance.is_valid())
			rs->instance_set_visible(link.mesh_instance, p_visible);
	}
}

void Rope::_clear_links() {
	auto rs = RenderingServer::get_singleton();
	auto ps = PhysicsServer3D::get_singleton();

	for (auto &link : _links) {
		if (link.mesh_instance.is_valid()) {
			rs->free_rid(link.mesh_instance);
			link.mesh_instance = RID();
		}

		if (link.physics_body.is_valid()) {
			ps->free_rid(link.physics_body);
			link.physics_body = RID();
		}
	}

	_links.clear();
}

void Rope::_rebuild_link_shape() {
	auto ps = PhysicsServer3D::get_singleton();
	int particle_count = get_particle_count_for_length();
	DEV_ASSERT(_links.size() == (particle_count - 1));

	// free old shape
	if (_link_shape.is_valid()) {
		ps->free_rid(_link_shape);
		_link_shape = RID();
	}

	// as capsules
	Dictionary capsule;
	float radius = get_rope_width() * 0.5f;
	float height = Math::max(get_rope_length() / (particle_count - 1), radius * 2.0f);
	capsule["radius"] = radius;
	capsule["height"] = height + radius * 2;

	// physics shape
	_link_shape = ps->capsule_shape_create();
	ps->shape_set_data(_link_shape, capsule);
	ps->shape_set_margin(_link_shape, radius / 10.0);

	// clear exclusion list and re-add all links with new shape
	_exclusion_list.clear();

#if ENABLE_COLLISION_LAYER
	for (int idx = 0; idx < _links.size(); idx++) {
		auto &link = _links[idx];
		ps->body_add_shape(link.physics_body, _link_shape, Transform3D());
		_exclusion_list.append(link.physics_body);
	}
#endif

	if (_ray_cast.is_valid())
		_ray_cast->set_exclude(_exclusion_list);
}

void Rope::_rebuild_links() {
	auto rs = RenderingServer::get_singleton();
	auto ps = PhysicsServer3D::get_singleton();
	Ref<World3D> w3d = get_world_3d();
	ERR_FAIL_NULL_MSG(rs, "RenderingServer missing");
	ERR_FAIL_NULL_MSG(w3d, "World3D missing");

	if (rs && w3d.is_valid()) {
		auto scenario = w3d->get_scenario();
		if (scenario.is_valid()) {
			RID space = w3d->get_space();

			RID mesh;
			if (_appearance.is_valid()) {
				auto mesh_ref = _appearance->get_array_mesh();
				mesh = mesh_ref.is_valid() ? mesh_ref->get_rid() : RID();
			}

			// clear old links
			_clear_links();

			// create one instance for each gap between particles
			// this will be one less than the particle count
			auto count = _particles.size() - 1;
			_links.resize(count);

			_calculate_links_for_particles();

			for (auto &link : _links) {
#if ENABLE_COLLISION_LAYER
				// create physics body
				link.physics_body = ps->body_create();
				if (link.physics_body.is_valid()) {
					ps->body_set_mode(link.physics_body, PhysicsServer3D::BODY_MODE_KINEMATIC);
					ps->body_set_space(link.physics_body, space);

					ps->body_set_collision_layer(link.physics_body, _collision_layer);
					ps->body_set_collision_mask(link.physics_body, _collision_mask);

					ps->body_set_state(link.physics_body, PhysicsServer3D::BODY_STATE_TRANSFORM, link.xform);

					ps->body_attach_object_instance_id(link.physics_body, get_instance_id());
					// shape is set by _rebuild_link_shape.
				}
#endif

				// have mesh? set it
				if (mesh.is_valid()) {
					link.mesh_instance = rs->instance_create2(mesh, scenario);
					if (link.mesh_instance.is_valid()) {
						rs->instance_attach_object_instance_id(link.mesh_instance, get_instance_id());
						rs->instance_set_visible(link.mesh_instance, is_visible_in_tree());
						rs->instance_set_transform(link.mesh_instance, link.xform);
					}
				}
			}

			// rebuild the shapes
			_rebuild_link_shape();
		}
	}
}

void Rope::_update_links() {
	auto rs = RenderingServer::get_singleton();
	auto ps = PhysicsServer3D::get_singleton();

	for (const auto &link : _links) {
#if ENABLE_COLLISION_LAYER
		if (link.physics_body.is_valid()) {
			ps->body_set_state(link.physics_body, PhysicsServer3D::BODY_STATE_TRANSFORM, link.xform);
		}
#endif

		if (link.mesh_instance.is_valid()) {
			rs->instance_set_transform(link.mesh_instance, link.xform);
		}
	}
}

#pragma endregion

#pragma region Rope Rebuilding

Vector3 _small_offset(float d) {
	float x = rand() % 2 ? -1.0 : 1.0;
	float y = rand() % 2 ? -1.0 : 1.0;
	float z = rand() % 2 ? -1.0 : 1.0;
	return Vector3(x * d, y * d, z * d);
}

void Rope::_rebuild_rope() {
	SCOPED_TIMER(_rebuild_rope);
	_is_rebuilding = true;

	// free previous shapes
	_clear_links();
	_frames.clear();

	auto global_position = get_global_position();
	const int previous_count = _particles.size();
	int particle_count = get_particle_count_for_length();

	// calculate our direction and previous position based upon
	// the end we're expanding from
	Vector3 direction = _gravity.normalized();
	Vector3 current_pos = global_position;
	if (previous_count >= 2) {
		if (_grow_from == Start) {
			current_pos = _particles[0].pos_cur;
			direction = current_pos.direction_to(_particles[1].pos_cur);
		} else {
			current_pos = _particles[previous_count - 1].pos_cur;
			direction = current_pos.direction_to(_particles[previous_count - 2].pos_cur);
		}
	} else {
		Vector3 start = global_position;
		Vector3 end = global_position;
		Transform3D xform;

		// do we have a start/end anchor?
		int anchor_count = get_anchor_count();
		int start_idx = anchor_count > 0 ? 0 : -1;
		int end_idx = anchor_count > 1 ? anchor_count - 1 : -1;

		if (_grow_from == Start) {
			if (start_idx >= 0)
				start = get_anchor_transform(start_idx).origin;
			if (end_idx >= 0)
				end = get_anchor_transform(end_idx).origin;
		} else {
			if (start_idx >= 0)
				end = get_anchor_transform(start_idx).origin;
			if (end_idx >= 0)
				start = get_anchor_transform(end_idx).origin;
		}
		current_pos = start;
		direction = (end - start).normalized();
		if (direction.length() == 0)
			direction = _gravity.normalized();
	}

	// grow
	auto segment_length = _rope_length / (particle_count - 1);
	float jitter_offset = segment_length / 10.0;
	Vector3 direction_segment = direction * segment_length;
	int insert_idx = 0;
	while (_particles.size() < particle_count) {
		Particle particle;
		// NOTE: this flag generates a small initial offset to the calculated previous position
		// so the simulation isn't "perfectly aligned" which can cause problems growing the rope
		// in a direction perfectly aligned with the gravity vector.
		//
		// If the row grows too slowly "stacking" can still occur as the stiffness will stablize it.
		Vector3 jitter_prev = (_jitter_initial_position ? _small_offset(jitter_offset) : Vector3());
		Vector3 jitter_cur = (_jitter_initial_position ? _small_offset(jitter_offset) : Vector3());
		particle.pos_prev = current_pos + jitter_prev;
		particle.pos_cur = current_pos + jitter_cur;
		particle.anchor_idx = -1;

		// for next particle
		auto jitter_dir = (direction + (_jitter_initial_position ? _small_offset(0.1) : Vector3())).normalized();
		current_pos = current_pos + direction_segment;

		if (_grow_from == Start)
			_particles.insert(insert_idx++, particle);
		else
			_particles.push_back(particle);

		// integrate while adding particles to help stabilize rope as it grows
		_stiff_rope(1);
	}

	// shrink
	if (particle_count > 0) {
		while (_particles.size() > particle_count) {
			if (_grow_from == Start)
				_particles.remove_at(0);
			else
				_particles.remove_at(_particles.size() - 1);

			// integrate while adding particles to help stabilize rope as it shrinks
			_stiff_rope(1);
		}
	}

	// initial rebuilds
	_rebuild_links();
	_rebuild_anchors();

	// only run preprocess on the first build
	if (previous_count == 0) {
		if (_particles.size() == _initial_pos.size()) {
			for (int i = 0; i < _particles.size(); i++) {
				_particles[i].pos_cur = to_global(_initial_pos[i]);
				_particles[i].pos_prev = _particles[i].pos_cur;
				_particles[i].accel = Vector3();
			}
		} else {
			const float preprocess_delta = 1.0 / _simulation_rate;
			const int preprocess_iterations = Math::max(int(_simulation_rate * _preprocess_time), 1);
			_update_physics(preprocess_delta, preprocess_iterations);
		}
	}

	_queue_redraw();
	_is_rebuilding = false;
}

void Rope::_queue_rope_rebuild() {
	ERR_FAIL_COND_EDMSG(_is_rebuilding, "Retrigger while Rope is already being rebuilt!");
	_rebuild = true;
}

bool Rope::_pop_rebuild() {
	bool should_rebuild = _rebuild;
	_rebuild = false;
	return should_rebuild;
}

void Rope::_bake_initial_pos() {
	_initial_pos.clear();
	// convert world-space particle positions to local space for storage.
	for (const auto &particle : _particles) {
		_initial_pos.push_back(to_local(particle.pos_cur));
	}
}

#pragma endregion

#pragma region Accessors

void Rope::set_rope_width(float val) {
	if (val != _rope_width) {
		_rope_width = val;
		_queue_rope_rebuild();
	}
}

float Rope::get_rope_width() const {
	return _rope_width;
}

// appearance passthrough accessors
#define APPEARENCE_ACCESSOR(m_type, m_name, m_default) \
	m_type Rope::get_##m_name() const { return _appearance != nullptr ? _appearance->get_##m_name() : m_default; }

APPEARENCE_ACCESSOR(float, rope_twist, 1.0)
APPEARENCE_ACCESSOR(int, rope_lod, 2)
APPEARENCE_ACCESSOR(Ref<Material>, material, nullptr)
APPEARENCE_ACCESSOR(float, start_offset, 0.0)
APPEARENCE_ACCESSOR(float, end_offset, 0.0)
#undef APPEARENCE_ACCESSOR

void Rope::set_particles_per_meter(float val) {
	if (val != _particles_per_meter) {
		_particles_per_meter = val;
		_queue_rope_rebuild();
	}
}

float Rope::get_particles_per_meter() const {
	return _particles_per_meter;
}

void Rope::_on_appearance_changed() {
	if (is_inside_tree()) {
		_queue_rope_rebuild();
	}
}

Ref<RopeAppearance> Rope::get_appearance() const {
	return _appearance;
}

void Rope::set_appearance(const Ref<RopeAppearance> &val) {
	if (_appearance != val) {
		// disconnect old
		if (_appearance != nullptr) {
			_appearance->disconnect("changed", Callable(this, "_on_appearance_changed"));
		}

		_appearance = val;

		// connect new
		if (_appearance != nullptr) {
			_appearance->connect("changed", Callable(this, "_on_appearance_changed"));
		}

		_queue_redraw();
	}
}

void Rope::_set_attachments_from_children() {
	// create an appearance if none is set
	if (_appearance.is_null()) {
		set_appearance(Ref<RopeAppearance>(memnew(RopeAppearance)));
	}

	const int child_count = get_child_count();
	for (int idx = 0; idx < child_count; idx++) {
		Node *child = get_child(idx);
		const NodePath path = get_path_to(child);

		// check if this child is already in the attachment list
		const int att_count = _appearance->get_attachment_count();
		bool already_added = false;
		for (int att_idx = 0; att_idx < att_count; att_idx++) {
			if (_appearance->get_attachment_nodepath(att_idx) == path) {
				already_added = true;
				break;
			}
		}

		if (!already_added) {
			_appearance->set_attachment_count(att_count + 1);
			_appearance->set_attachment_nodepath(att_count, path);
		}
	}
}

uint64_t Rope::get_particle_count() const {
	return _particles.size();
}

Ref<ArrayMesh> Rope::get_baked_mesh() const {
	if (_rope_mesh.is_valid()) {
		_rope_mesh->_update_mesh_internal(_frames, get_material());
		return _rope_mesh;
	}

	return Ref<ArrayMesh>();
}

void Rope::set_liquid_area(LiquidArea *liquid_area) {
	_liquid_area_id = liquid_area ? liquid_area->get_instance_id() : 0;
	_queue_redraw();
}

LiquidArea *Rope::get_liquid_area() const {
	return Object::cast_to<LiquidArea>(ObjectDB::get_instance(_liquid_area_id));
}

float Rope::get_current_rope_length() const {
	float length = 0.0;
	for (int i = 0; i < _particles.size() - 1; i++)
		length += (_particles[i + 1].pos_cur - _particles[i].pos_cur).length();
	return length;
}

int Rope::get_rope_sides() const {
	if (_appearance == nullptr)
		return 6;
	else {
		auto sides = _appearance->get_rope_sides();
		if (sides >= 3 && sides <= MAX_ROPE_SIDES) {
			return sides;
		} else {
			auto diameter = get_rope_width();
			const float ratio = UtilityFunctions::ease(diameter, .2);
			return Math::clamp(int(Math::lerp(3, 12, ratio)), 3, 12);
		}
	}
}

int Rope::get_link_count() const {
	return _links.size();
}

Transform3D Rope::get_link(int index) const {
	if (index < 0 || index >= _links.size())
		return Transform3D();

	return _links[index].xform;
}

TypedArray<Transform3D> Rope::get_all_links() const {
	TypedArray<Transform3D> array;
	for (const auto &link : _links)
		array.push_back(link.xform);
	return array;
}

int Rope::get_rope_frame_count() const {
	return _frames.size();
}

Transform3D Rope::get_rope_frame(int index) const {
	if (index < 0 || index >= _frames.size())
		return Transform3D();

	return _frames[index];
}

TypedArray<Transform3D> Rope::get_all_rope_frames() const {
	TypedArray<Transform3D> array;
	for (const auto &frame : _frames)
		array.push_back(frame);
	return array;
}

TypedArray<Vector3> Rope::get_particle_positions() const {
	TypedArray<Vector3> array;

	// manual copy
	for (const auto &particle : _particles)
		array.push_back(particle.pos_cur);
	return array;
}

PackedVector3Array Rope::_get_initial_pos() const {
	PackedVector3Array result;
	result.resize(_initial_pos.size());
	for (uint32_t idx = 0; idx < _initial_pos.size(); idx++) {
		result[idx] = _initial_pos[idx];
	}
	return result;
}

void Rope::_set_initial_pos(const PackedVector3Array &val) {
	_initial_pos.resize(val.size());
	for (int idx = 0; idx < val.size(); idx++) {
		_initial_pos[idx] = val[idx];
	}
}

uint64_t Rope::get_particle_count_for_length() const {
	int particle_count = uint64_t(get_rope_length() * get_particles_per_meter()) + 1;
	particle_count = Math::max(particle_count, 2);
	return particle_count;
}

#pragma endregion

#pragma region Anchor Subclassing
void Rope::_notify_anchors_changed() {
	if (!GDVIRTUAL_IS_OVERRIDDEN(_update_anchors)) {
		notify_property_list_changed();
	}
	_anchors_dirty = true;
}

void Rope::_update_anchors() const {
	if (!is_inside_tree())
		return;

	if (GDVIRTUAL_IS_OVERRIDDEN(_update_anchors)) {
		GDVIRTUAL_CALL(_update_anchors);
	}
}

#pragma endregion

#pragma region Attachment Subclassing

void Rope::_notify_attachments_changed() {
	if (_appearance.is_valid()) {
		_appearance->notify_property_list_changed();
	}
	_attachments_dirty = true;
}

void Rope::_update_attachments() const {
	if (GDVIRTUAL_IS_OVERRIDDEN(_update_attachments)) {
		GDVIRTUAL_CALL(_update_attachments);
	}
}

Transform3D Rope::_get_attachment_local_transform(int attach_idx) const {
	Transform3D xform;
	if (GDVIRTUAL_IS_OVERRIDDEN(_get_attachment_local_transform)) {
		GDVIRTUAL_CALL(_get_attachment_local_transform, attach_idx, xform);
	}
	return xform;
}

#pragma endregion

#pragma region Anchors Property List

// Build up a dynamic list of properties for each element in the position vector.
void Rope::_get_property_list(List<PropertyInfo> *p_list) const {
	ERR_FAIL_NULL(p_list);

	// if anchors are determined by subclassing, return nothing here
	if (GDVIRTUAL_IS_OVERRIDDEN(_update_anchors)) {
		return;
	}

	// new layout
	LocalVector<PropertyInfo> props;
	for (uint32_t i = 0; i < _anchors.size(); i++) {
		uint32_t usage = PROPERTY_USAGE_DEFAULT;
		String path = ANCHORS_KEY + itos(i) + "/";

		// uniform doesn't use offset
		usage = (_anchor_distribution == Distribution::UNIFORM || _anchor_distribution == Distribution::REAL)
				? PROPERTY_USAGE_NONE
				: PROPERTY_USAGE_DEFAULT;
		p_list->push_back(PropertyInfo(Variant::FLOAT, path + OFFSET_KEY, PROPERTY_HINT_NONE, "suffix:m", usage));

		// Only absolute uses the from end/start
		usage = _anchor_distribution == Distribution::ABSOLUTE ? PROPERTY_USAGE_DEFAULT : PROPERTY_USAGE_NONE;
		p_list->push_back(PropertyInfo(Variant::INT, path + FROM_KEY, PROPERTY_HINT_ENUM, FROM_HINT, usage));

		p_list->push_back(PropertyInfo(Variant::NODE_PATH, path + NODE_PATH_KEY,
				PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Rope,RopeAnchor,RigidBody3D"));

		// if node_path points to a RopeAnchor, hide the behavior property
		NodePath node_path = _anchors[i].node_path;
		Node *node = get_node_or_null(node_path);
		Rope *as_rope = Object::cast_to<Rope>(node);
		RopeAnchor *as_anchor = Object::cast_to<RopeAnchor>(node);
		RigidBody3D *as_rigid = Object::cast_to<RigidBody3D>(node);

		// hide behavior if we can get it from anchor
		usage = (as_anchor || as_rope)
				? PROPERTY_USAGE_NONE
				: PROPERTY_USAGE_DEFAULT;
		p_list->push_back(PropertyInfo(Variant::INT, path + BEHAVIOR_KEY, PROPERTY_HINT_ENUM, BEHAVIOR_HINT, usage));

		// hide friction if we can get it from anchor
		usage = as_anchor
				? PROPERTY_USAGE_NONE
				: PROPERTY_USAGE_DEFAULT;
		p_list->push_back(PropertyInfo(Variant::FLOAT, path + FRICTION_KEY, PROPERTY_HINT_RANGE, "0.0,1.0,0.01", usage));

		// hide position if we can get it from node3d
		usage = Object::cast_to<Node3D>(node)
				? PROPERTY_USAGE_NONE
				: PROPERTY_USAGE_DEFAULT;
		p_list->push_back(PropertyInfo(Variant::VECTOR3, path + POSITION_KEY, PROPERTY_HINT_NONE, "suffix:m", usage));
	}
}

bool Rope::_set(const StringName &p_name, const Variant &p_property) {
	// if anchors are determined by subclassing, return nothing here
	if (GDVIRTUAL_IS_OVERRIDDEN(_update_anchors)) {
		return false;
	}

	String path = p_name;
	if (path.begins_with(ANCHORS_KEY)) {
		int which = path.get_slicec('/', 1).to_int();
		String what = path.get_slicec('/', 2);
		ERR_FAIL_INDEX_V(which, (int)_anchors.size(), false);

		if (what == OFFSET_KEY) {
			set_anchor_offset(which, (float)p_property);
		} else if (what == FROM_KEY) {
			set_anchor_from(which, (int)p_property);
		} else if (what == NODE_PATH_KEY) {
			set_anchor_nodepath(which, (NodePath)p_property);
		} else if (what == BEHAVIOR_KEY) {
			set_anchor_behavior(which, (AnchorBehavior)(int)p_property);
		} else if (what == FRICTION_KEY) {
			set_anchor_friction(which, (float)p_property);
		} else if (what == POSITION_KEY) {
			Vector3 pos = (Vector3)p_property;
			set_anchor_transform(which, Transform3D(Basis(), pos));
		} else {
			return false;
		}
		return true;
	}

	return false;
}

bool Rope::_get(const StringName &p_name, Variant &r_property) const {
	// if anchors are determined by subclassing, return nothing here
	if (GDVIRTUAL_IS_OVERRIDDEN(_update_anchors)) {
		return false;
	}

	String path = p_name;
	if (path.begins_with(ANCHORS_KEY)) {
		int which = path.get_slicec('/', 1).to_int();
		String what = path.get_slicec('/', 2);
		ERR_FAIL_INDEX_V(which, (int)_anchors.size(), false);

		if (what == OFFSET_KEY) {
			r_property = get_anchor_offset(which);
		} else if (what == FROM_KEY) {
			r_property = get_anchor_from(which);
		} else if (what == NODE_PATH_KEY) {
			r_property = (NodePath)get_anchor_nodepath(which);
		} else if (what == BEHAVIOR_KEY) {
			r_property = (int)get_anchor_behavior(which);
		} else if (what == FRICTION_KEY) {
			r_property = get_anchor_friction(which);
		} else if (what == POSITION_KEY) {
			r_property = get_anchor_transform(which).origin;
		} else {
			return false;
		}
		return true;
	}

	return false;
}

#pragma endregion

#pragma region Anchors Properties

void Rope::set_anchor_count(int count) {
	ERR_FAIL_COND(count < 0);
	if (_anchors.size() != count) {
		_anchors.resize(count);
		_notify_anchors_changed();
	}
}

int Rope::get_anchor_count() const {
	return (int)_anchors.size();
}

void Rope::clear_anchors() {
	_anchors.clear();
	_notify_anchors_changed();
}

void Rope::set_anchor_offset(int idx, float offset) {
	ERR_FAIL_INDEX(idx, (int)_anchors.size());
	if (_anchors[idx].offset != offset) {
		_anchors[idx].offset = offset;
		_notify_anchors_changed();
	}
}

float Rope::get_anchor_offset(int idx) const {
	ERR_FAIL_INDEX_V(idx, (int)_anchors.size(), 0.0f);
	return _anchors[idx].offset;
}

void Rope::set_anchor_from(int idx, int from) {
	ERR_FAIL_INDEX(idx, (int)_anchors.size());
	if (_anchors[idx].from_end != (from == End)) {
		_anchors[idx].from_end = (from == End);
		_notify_anchors_changed();
	}
}

int Rope::get_anchor_from(int idx) const {
	ERR_FAIL_INDEX_V(idx, (int)_anchors.size(), Start);
	return _anchors[idx].from_end ? End : Start;
}

void Rope::set_anchor_nodepath(int idx, const NodePath &path) {
	ERR_FAIL_INDEX(idx, (int)_anchors.size());
	if (_anchors[idx].node_path != path) {
		_anchors[idx].node_path = path;
		_notify_anchors_changed();
	}

	if (is_inside_tree()) {
		Node3D *node = Object::cast_to<Node3D>(get_node_or_null(path));
		if (node) {
			_anchors[idx].node_id = node->get_instance_id();

			_anchors[idx].transform = node->get_global_transform();
			RigidBody3D *rigid_body = Object::cast_to<RigidBody3D>(node->get_parent());
			if (rigid_body) {
				_anchors[idx].rigid_body_id = rigid_body->get_instance_id();
			} else {
				_anchors[idx].rigid_body_id = 0;
			}

			RopeAnchor *anchor = Object::cast_to<RopeAnchor>(node);
			if (anchor) {
				_anchors[idx].behavior = (AnchorBehavior)anchor->get_behavior();
			}

		} else {
			// not found? clear both
			_anchors[idx].node_id = 0;
			_anchors[idx].rigid_body_id = 0;
		}
	}

	_notify_anchors_changed();
}

NodePath Rope::get_anchor_nodepath(int idx) const {
	ERR_FAIL_INDEX_V(idx, (int)_anchors.size(), NodePath());
	return _anchors[idx].node_path;
}

void Rope::set_anchor_behavior(int idx, AnchorBehavior behavior) {
	ERR_FAIL_INDEX(idx, (int)_anchors.size());
	if (_anchors[idx].behavior != behavior) {
		_anchors[idx].behavior = behavior;
		_notify_anchors_changed();
	}
}

AnchorBehavior Rope::get_anchor_behavior(int idx) const {
	ERR_FAIL_INDEX_V(idx, (int)_anchors.size(), AnchorBehavior::ANCHORED);

	// use the node transform if available
	RopeAnchor *anchor = Object::cast_to<RopeAnchor>(_anchors[idx].get_node());
	if (anchor) {
		return (AnchorBehavior)anchor->get_behavior();
	}

	return _anchors[idx].behavior;
}

void Rope::set_anchor_friction(int idx, float friction) {
	ERR_FAIL_INDEX(idx, (int)_anchors.size());
	if (_anchors[idx].friction != friction) {
		_anchors[idx].friction = friction;
		_notify_anchors_changed();
	}
}

float Rope::get_anchor_friction(int idx) const {
	ERR_FAIL_INDEX_V(idx, (int)_anchors.size(), 0.5f);

	RopeAnchor *anchor = Object::cast_to<RopeAnchor>(_anchors[idx].get_node());
	if (anchor) {
		return anchor->get_friction();
	}

	return _anchors[idx].friction;
}

void Rope::set_anchor_transform(int idx, const Transform3D &transform) {
	ERR_FAIL_INDEX(idx, (int)_anchors.size());
	if (_anchors[idx].transform != transform) {
		_anchors[idx].transform = transform;
		_notify_anchors_changed();
	}
}

Transform3D Rope::get_anchor_transform(int idx) const {
	ERR_FAIL_INDEX_V(idx, (int)_anchors.size(), Transform3D());

	// use the node transform if available
	Node3D *node = Object::cast_to<Node3D>(_anchors[idx].get_node());
	if (node) {
		return node->get_global_transform();
	}

	return _anchors[idx].transform;
}

void Rope::set_anchor_rigidbody(int idx, RigidBody3D *body) {
	ERR_FAIL_INDEX(idx, (int)_anchors.size());

	if (body == nullptr && _anchors[idx].rigid_body_id != 0) {
		_anchors[idx].rigid_body_id = 0;
		_notify_anchors_changed();
		return;
	}

	uint64_t current_id = body->get_instance_id();
	if (_anchors[idx].rigid_body_id != current_id) {
		_anchors[idx].rigid_body_id = current_id;
		_notify_anchors_changed();
	} else {
		return;
	}
}

RigidBody3D *Rope::get_anchor_rigidbody(int idx) const {
	ERR_FAIL_INDEX_V(idx, (int)_anchors.size(), nullptr);
	return Object::cast_to<RigidBody3D>(_anchors[idx].get_rigid_body());
}

float Rope::get_anchor_abs_offset(int idx) const {
	ERR_FAIL_INDEX_V(idx, (int)_anchors.size(), 0.0f);
	return _anchors[idx]._abs_offset;
}

int Rope::get_anchor_particle_count(int anchor_idx) const {
	ERR_FAIL_INDEX_V(anchor_idx, (int)_anchors.size(), 0);
	RopeAnchor *rope_anchor = _get_rope_anchor(anchor_idx);
	if (rope_anchor) {
		return rope_anchor->get_particle_count();
	}
	// non-RopeAnchor anchors occupy a single particle
	return 1;
}

Vector3 Rope::get_anchor_particle_position(int anchor_idx, int particle_idx) const {
	ERR_FAIL_INDEX_V(anchor_idx, (int)_anchors.size(), Vector3());
	RopeAnchor *rope_anchor = _get_rope_anchor(anchor_idx);
	if (rope_anchor) {
		return rope_anchor->get_particle_position(particle_idx);
	}
	// non-RopeAnchor anchors have a single particle at the anchor origin
	return Vector3();
}

void Rope::set_anchor_distribution(int val) {
	const Distribution new_dist = (Distribution)val;
	if (new_dist == _anchor_distribution) {
		return;
	}

	// Remap existing offset values through _abs_offset so anchors stay at the same
	// rope positions after the mode change.
	if (!_anchors.is_empty()) {
		const float rope_len = get_rope_length();

		switch (new_dist) {
			case Distribution::ABSOLUTE:
				// preserve from_end direction
				for (auto &anchor : _anchors) {
					anchor.offset = anchor.from_end ? (rope_len - anchor._abs_offset) : anchor._abs_offset;
				}
				break;
			case Distribution::RELATIVE:
				// offset[0] = _abs_offset[0]; offset[i] = _abs_offset[i] - offset[i-1]
				for (int idx = 0; idx < (int)_anchors.size(); idx++) {
					float prev_offset = (idx > 0) ? _anchors[idx - 1]._abs_offset : 0.0f;
					_anchors[idx].offset = _anchors[idx]._abs_offset - prev_offset;
				}
				break;
			case Distribution::SCALAR:
				for (auto &anchor : _anchors) {
					anchor.offset = (rope_len > 0.0f) ? (anchor._abs_offset / rope_len) : 0.0f;
				}
				break;
			case Distribution::UNIFORM:
				break;
			case Distribution::REAL:
				// _abs_offset is computed from anchor transforms in _rebuild_anchors; nothing to remap
				break;
		}
	}

	_anchor_distribution = new_dist;
	_notify_anchors_changed();
}

int Rope::get_anchor_distribution() const {
	return (int)_anchor_distribution;
}

#pragma endregion

#pragma region Anchors Rebuilding

// Update anchor fields, and especially abs_offset.
void Rope::_rebuild_anchors() {
	SCOPED_TIMER(_rebuild_anchors);

	// exit early if not inside tree
	ERR_FAIL_COND(!is_inside_tree());

	// update the settings from the current scene
	for (int idx = 0; idx < _anchors.size(); idx++) {
		auto &anchor = _anchors[idx];
		Node3D *node = Object::cast_to<Node3D>(get_node_or_null(anchor.node_path));
		if (node) {
			anchor.node_id = node->get_instance_id();

			// if we have a node, set transform from that
			anchor.transform = node->get_global_transform();

			// if parent is a RigidBody3D, set that.
			RigidBody3D *rigid_body = Object::cast_to<RigidBody3D>(node->get_parent());
			if (rigid_body) {
				anchor.rigid_body_id = rigid_body->get_instance_id();
			} else {
				anchor.rigid_body_id = 0;
			}

			// if node is a RigidBody3D, set that instead.
			RigidBody3D *self_rigid_body = Object::cast_to<RigidBody3D>(node);
			if (self_rigid_body) {
				anchor.rigid_body_id = self_rigid_body->get_instance_id();
			}

			// if node is an anchor, set behavior from it.
			RopeAnchor *anchor_obj = Object::cast_to<RopeAnchor>(node);
			if (anchor_obj) {
				anchor.behavior = (AnchorBehavior)anchor_obj->get_behavior();
			}
		}

		// update _abs_offset based on distribution
		switch (_anchor_distribution) {
			case Distribution::ABSOLUTE:
				if (anchor.from_end) {
					anchor._abs_offset = get_rope_length() - anchor.offset;
				} else {
					anchor._abs_offset = anchor.offset;
				}
				break;
			case Distribution::RELATIVE:
				if (idx > 0) {
					anchor._abs_offset = _anchors[idx].offset + _anchors[idx - 1]._abs_offset;
				} else {
					anchor._abs_offset = anchor.offset;
				}
				break;
			case Distribution::UNIFORM:
				anchor._abs_offset = get_rope_length() * (float(idx) / float(_anchors.size() - 1));
				anchor.offset = anchor._abs_offset;
				break;
			case Distribution::SCALAR:
				anchor._abs_offset = get_rope_length() * anchor.offset;
				break;
			case Distribution::REAL:
				// Cumulative straight-line distance between consecutive anchor origins.
				if (idx == 0) {
					anchor._abs_offset = 0.0f;
				} else {
					anchor._abs_offset = _anchors[idx - 1]._abs_offset + _anchors[idx - 1].transform.origin.distance_to(anchor.transform.origin);
				}
				break;
		}
	}

	// sort anchors by _abs_offset
	if (_anchor_distribution == Distribution::ABSOLUTE || _anchor_distribution == Distribution::SCALAR) {
		_anchors.sort();
	}
}

int Rope::_get_particle_for_offset(float offset) const {
	float position = offset / _rope_length;
	int last = _particles.size() - 1;

	// round to nearest particle index
	return Math::clamp((int)Math::round(last * position), 0, last);
}

#define VALID_PARTICLE_IDX(idx) (idx >= 0 && idx < (int)_particles.size())
#define INVALID_PARTICLE_IDX(idx) (!VALID_PARTICLE_IDX(idx))
#define VALID_ANCHOR_IDX(idx) (idx >= 0 && idx < (int)_anchors.size())
#define INVALID_ANCHOR_IDX(idx) (!VALID_ANCHOR_IDX(idx))

RopeAnchor *Rope::_get_rope_anchor(int anchor_idx) const {
	if (anchor_idx < 0 || anchor_idx >= (int)_anchors.size()) {
		return nullptr;
	}
	return Object::cast_to<RopeAnchor>(_anchors[anchor_idx].get_node());
}

bool Rope::_is_anchor_free(int anchor_idx, int anchor_count) const {
	// index out of range? default to free
	if (anchor_idx < 0 || anchor_idx >= anchor_count) {
		return true;
	}
	return get_anchor_behavior(anchor_idx) == AnchorBehavior::FREE;
}

bool Rope::_is_anchor_fixed(int anchor_idx, int anchor_count) const {
	// index out of range? not fixed
	if (anchor_idx < 0 || anchor_idx >= anchor_count) {
		return false;
	}
	auto behavior = get_anchor_behavior(anchor_idx);
	return behavior == AnchorBehavior::ANCHORED || behavior == AnchorBehavior::GUIDED;
}

bool Rope::_is_anchor_moving(int anchor_idx, int anchor_count) const {
	// index out of range? not moving
	if (anchor_idx < 0 || anchor_idx >= anchor_count) {
		return false;
	}
	auto behavior = get_anchor_behavior(anchor_idx);
	return behavior == AnchorBehavior::SLIDING || behavior == AnchorBehavior::TOWING;
}

bool Rope::_is_rope_sliding(int anchor_idx, int anchor_count) const {
	// index out of range? not sliding
	if (anchor_idx < 0 || anchor_idx >= anchor_count) {
		return false;
	}
	auto behavior = get_anchor_behavior(anchor_idx);
	return behavior == AnchorBehavior::SLIDING || behavior == AnchorBehavior::GUIDED;
}

int Rope::_particle_for_anchor(int p_anchor_idx) const {
	if (p_anchor_idx < 0 || p_anchor_idx >= (int)_anchors.size()) {
		return -1;
	}
	return _anchors[p_anchor_idx].particle_idx;
}

int Rope::_anchor_for_particle(int p_particle_idx) const {
	if (p_particle_idx < 0 || p_particle_idx >= (int)_particles.size()) {
		return -1;
	}
	return _particles[p_particle_idx].anchor_idx;
}

float Rope::_particle_stretch(int particle_idx) const {
	if (particle_idx < 0 || particle_idx >= (int)_particles.size() - 1) {
		return -1.0;
	}

	return _particles[particle_idx].stretch;
}

#pragma endregion

#pragma region Frame Calculations

float Rope::_get_average_segment_length() const {
	// segment count is N-1 particle count
	return get_rope_length() / float(_particles.size() - 1);
}

PackedVector3Array Rope::_get_control_points_for_particle(int index) const {
	auto segment_length = _get_average_segment_length();
	const auto last_index = _particles.size() - 1;
	const auto projected = (_particles[index].T * segment_length);

	PackedVector3Array p;
	p.resize(4);

	// at index 0 project a fictitious previous point
	if (index > 0)
		p[0] = _particles[index - 1].pos_cur;
	if (index == 0)
		p[0] = _particles[index].pos_cur - projected;

	// p[1] is the current point at index
	p[1] = _particles[index].pos_cur;

	if (index < last_index)
		p[2] = _particles[index + 1].pos_cur;
	else
		p[2] = _particles[index].pos_cur + projected;

	// last
	if (index < last_index - 1)
		p[3] = _particles[index + 2].pos_cur;
	else if (index < last_index)
		p[3] = _particles[index + 1].pos_cur + projected;
	else
		p[3] = _particles[index].pos_cur + projected * 2.0;
	return p;
}

Pair<Vector3, Vector3> Rope::_catmull_interpolate(const Vector3 &p0, const Vector3 &p1, const Vector3 &p2, const Vector3 &p3, float tension, float t) const {
	float t_sqr = t * t;
	float t_cube = t_sqr * t;

	Vector3 m1 = (1.0 - tension) / 2.0 * (p2 - p0);
	Vector3 m2 = (1.0 - tension) / 2.0 * (p3 - p1);

	Vector3 a = (2.0 * (p1 - p2)) + m1 + m2;
	Vector3 b = (-3.0 * (p1 - p2)) - (2.0 * m1) - m2;

	Vector3 point = (a * t_cube) + (b * t_sqr) + (m1 * t) + p1;
	Vector3 tangent = ((3.0 * a * t_sqr) + (2.0 * b * t) + m1).normalized();

	return { point, tangent };
}

void Rope::_compute_particle_normals() {
	if (_particles.size()) {
		auto origin = get_global_position();

		auto start = _particles[0];
		start.T = (_particles[1].pos_cur - start.pos_cur).normalized();
		start.N = (start.pos_cur - origin).normalized();
		start.B = start.N.cross(start.T).normalized();
		_particles[0] = start;

		auto end_index = _particles.size() - 1;
		auto end = _particles[end_index];
		end.T = (end.pos_cur - _particles[end_index - 1].pos_cur).normalized();
		end.N = (end.pos_cur - origin).normalized();
		end.B = end.N.cross(end.T).normalized();
		_particles[end_index] = end;

		for (int i = 1; i < _particles.size() - 1; i++) {
			auto particle = _particles[i];
			particle.T = (_particles[i + 1].pos_cur - _particles[i - 1].pos_cur).normalized();
			particle.N = (particle.pos_cur - origin).normalized();
			particle.B = particle.N.cross(particle.T).normalized();
			_particles[i] = particle;
		}
	}
}

// compute a stable normal/binormal frame using parallel transport to reduce twisting
// ref: https://en.wikipedia.org/wiki/Parallel_transport
void Rope::_compute_parallel_transport(LocalVector<Transform3D> &frames) const {
	auto count = frames.size();
	if (count > 0) {
		// initial tangent
		Vector3 T = frames[0].basis.get_column(Y).normalized();

		// // use the basis for our rope start aligned to the first tangent
		// // as the reference frame. this keeps the normals and U coordinate
		// // from swinging wildly when the curvature changes orientation
		Basis global_basis = get_global_basis();
		auto basis_y = -global_basis.get_column(Y);
		global_basis.rotate_to_align(basis_y, T);
		Vector3 ref = global_basis.get_column(Y);

		// FIXME: this will spin wildly when tilted 90 off x & y axis.

		Vector3 N = global_basis.get_column(X);
		Vector3 B = global_basis.get_column(Z);

		frames[0].basis.set_column(X, N);
		frames[0].basis.set_column(Z, B);

		// parallel transport subsequent normals
		for (int i = 1; i < count; i++) {
			T = frames[i].basis.get_column(Y).normalized();
			auto prev_N = frames[i - 1].basis.get_column(X);

			// project previous normal onto plane perpendicular to new tangent
			auto proj = prev_N - T * prev_N.dot(T);
			if (Math::is_zero_approx(proj.length())) { // degenerate case, reproject using ref fallback
				proj = ref - T * ref.dot(T);
				if (Math::is_zero_approx(proj.length())) {
					proj = prev_N;
				}
			}

			N = proj.normalized();
			B = T.cross(N).normalized();

			frames[i].basis.set_column(X, N);
			frames[i].basis.set_column(Z, B);
		}
	}
}

int Rope::_frame_at_offset(const LocalVector<Transform3D> &frames, float offset, bool from_end) const {
	// find the nearest transform frame for the given offset
	float frames_per_segment = int(frames.size() / float(_particles.size()));
	float offset_particle = offset * get_particles_per_meter();
	float offset_frame = offset_particle * frames_per_segment;

	// if offset is negative work backwards from the end
	if (from_end)
		offset_frame = (frames.size() - 1) + offset_frame;

	// clamp in range
	offset_frame = Math::clamp(int(offset_frame), 0, int(frames.size() - 1));
	return offset_frame;
}

void Rope::_calculate_frames_for_particles(LocalVector<Transform3D> &frames) const {
	DEV_ASSERT(_particles.size() >= 2);

	Vector3 global_position = get_global_position();
	auto lod = get_rope_lod();
	if (lod < 1)
		lod = 1;

	// got to N-1 because we sample between pairs of particles
	frames.clear();
	for (int i = 0; i < _particles.size(); i++) {
		// 	auto step := get_draw_subdivision_step(camera_position, i);
		const auto particles = _get_control_points_for_particle(i);
		const auto &p0 = particles[0];
		const auto &p1 = particles[1];
		const auto &p2 = particles[2];
		const auto &p3 = particles[3];

		// sample out the catmull spline into points and tangents
		float t = 0.0;
		for (int j = 0; j < lod; j++) {
			auto current_result = _catmull_interpolate(p0, p1, p2, p3, 0.0, t);
			auto position = current_result.first;
			auto tangent = current_result.second;

			// store sampled position and tangent; compute a stable frame after sampling
			// Normal and Binormal are calculated in parallel transport propigation.
			position -= global_position;
			Vector3 zero = Vector3(0, 0, 0);
			frames.push_back(Transform3D(Basis(zero, tangent, zero), position));
			t += 1.0 / float(lod);

			// skip lod frames for the last particle, as it's the terminal frame
			if (i == _particles.size() - 1)
				break;
		}
	}

	// clip the frames by the start and end offsets
	// offset is scaled by rope width so the offset position is independent of rope size
	auto start_offset = get_start_offset() * get_rope_width();
	Basis start_basis = frames[0].basis;
	while (start_offset > 0.0 && frames.size() > 2) {
		// distance to next frame
		Transform3D &frame = frames[0];
		const Transform3D &next = frames[1];
		float dist = frame.origin.distance_to(next.origin);

		// if distance is less than the offset, remove the frame and continue
		if (dist < start_offset) {
			start_offset -= dist;
			frames.remove_at(0);
		} else {
			// move the frame along the line to the next frame
			Vector3 dir = (next.origin - frame.origin).normalized();
			frame.origin += dir * start_offset;
			frame.basis = start_basis;
			break;
		}
	}

	// clip the end offset by walking in reverse.
	auto end_offset = get_end_offset() * get_rope_width();
	Basis end_basis = frames[frames.size() - 1].basis;
	while (end_offset > 0.0 && frames.size() > 2) {
		// distance to next frame
		Transform3D &frame = frames[frames.size() - 1];
		const Transform3D &prev = frames[frames.size() - 2];
		float dist = frame.origin.distance_to(prev.origin);

		// if distance is less than the offset, remove the frame and continue
		if (dist < end_offset) {
			end_offset -= dist;
			frames.remove_at(frames.size() - 1);
		} else {
			// move the frame along the line to the previous frame
			Vector3 dir = (prev.origin - frame.origin).normalized();
			frame.origin += dir * end_offset;
			frame.basis = end_basis;
			break;
		}
	}

	// Need at least two frames to draw a rope
	if (frames.size() >= 2) {
		_compute_parallel_transport(frames);
	}
}

// calculate the transforms for each link so they can be used for both chains and capsule positioning
void Rope::_calculate_links_for_particles() {
	SCOPED_TIMER(_calculate_links_for_particles);

	DEV_ASSERT(_particles.size() >= 2);
	DEV_ASSERT(_links.size() == _particles.size() - 1);

	// need at least two particles to make a link
	if (_particles.size() >= 2) {
		// N-1 chain links per paricles
		auto count = _particles.size() - 1;
		for (int idx = 0; idx < count; idx++) {
			Vector3 pt = _particles[idx].pos_cur;
			Vector3 pt2 = _particles[idx + 1].pos_cur;
			Vector3 dir = (pt2 - pt);
			float dist = dir.length();
			if (dist > 0.0) {
				dir.normalize();

				// xform is at the midpoint between particles
				Transform3D xform(Basis(), pt + dir * dist / 2.0);
				xform.basis.rotate_to_align(Vector3(0, 1, 0), dir);

				// every other frame is rotated along the tangent 90deg
				// so that the links alternate
				if (idx % 2)
					xform = xform.rotated_local(Vector3(0, 1, 0), Math_PI / 2.0);

				// push back the unscaled xform
				// ...doubles this functions time but we can get tiny scale artifacts from the rotation
				// which will cause problems with jolt over time in certain edge conditions.
				// xform.orthogonalize();
				_links[idx].xform = xform;
			} else {
				// zero length link... either because the points are too close or coincidentally overlapping.
				// just push back the xform for the starting particle. There's no orientation we can align to,
				// but at least it will be in the right place if the particles separate in the next frame.
				_links[idx].xform = Transform3D(Basis(), pt);
			}
		}
	}
}

#pragma endregion

#pragma region Mesh Rendering
void Rope::_align_attachment_node(const NodePath &path, Transform3D xform, float offset) {
	auto diameter = get_rope_width();
	auto cap_scale = Vector3(diameter, diameter, diameter);
	Node *node = get_node_or_null(path);
	if (node && node->is_class("Node3D")) {
		Node3D *node3d = (Node3D *)node;
		if (node3d->is_visible()) {
			xform.origin += xform.basis.get_column(Y).normalized() * offset;

			node3d->set_global_transform(get_global_transform() * xform.orthonormalized());
			node3d->set_scale(cap_scale);
		}
	}
}

void Rope::_update_aabb() {
	AABB aabb;
	// update mesh and visual instance
	if (_rope_mesh.is_valid()) {
		aabb = _rope_mesh->get_custom_aabb();
	}
	set_custom_aabb(aabb);
}

float Rope::_lod_factor() const {
	// Compute the distance from the main camera to the rope's surface.
	// Build an AABB from particle world positions, expand by rope radius,
	// then measure the shortest distance from the camera to the AABB surface.
	float lod_factor = 1.0f;

	// need at least one particle to compute bounds
	if (_particles.size() == 0)
		return lod_factor;

	// find camera
	Camera3D *cam = nullptr;
	if (Engine::get_singleton()->is_editor_hint()) {
		EditorInterface *interface = EditorInterface::get_singleton();
		SubViewport *viewport = interface->get_editor_viewport_3d(0);
		if (viewport)
			cam = viewport->get_camera_3d();

	} else {
		Viewport *viewport = get_viewport();
		if (viewport)
			cam = viewport->get_camera_3d();
	}
	if (!cam)
		return lod_factor;

	// if camera is orthogonal just return a small constant
	if (cam->get_projection() == Camera3D::PROJECTION_ORTHOGONAL)
		return 1.0f;

	float multiplier = cam->get_camera_projection().get_lod_multiplier();
	Vector3 camera_position = cam->get_global_position();

	// compute aabb from particle positions (world space)
	AABB aabb = get_custom_aabb();
	Vector3 aabb_min = to_global(aabb.position);
	Vector3 aabb_max = to_global(aabb.position + aabb.size);

	// use the rope width as model size basis so that the length of rope doesn't affect LOD
	Vector3 surface_distance = Vector3(0.0, 0.0, 0.0).max(aabb_min - camera_position).max(camera_position - aabb_max);
	float surf_distance = surface_distance.length() * multiplier;
	float model_size = get_rope_width();

	// this 32 is a fudge factor based on taste, i'd like it to be based on something more concrete...
	lod_factor = 1.0 / surf_distance * model_size * get_lod_bias() * 16.0;
	return lod_factor;
}

void Rope::_draw_rope() {
	_update_aabb();

	// Bail early if there are no particles in rope.
	// This can happen if we get a draw call before the Rope is ready
	if (_particles.size() == 0) {
		if (_rope_mesh.is_valid()) {
			_rope_mesh->clear_mesh();
		}
		return;
	}

	// recompute normal, tangents, and binormals
	_compute_particle_normals();

	// recalculate the LOD frames
	_calculate_frames_for_particles(_frames);

	// Need at least two frames to draw a rope
	if (_frames.size() < 2) {
		if (_rope_mesh.is_valid()) {
			_rope_mesh->clear_mesh();
		}
		return;
	}

	auto diameter = get_rope_width();
	const auto radius = diameter * 0.5;

	// tube parameters. set the number of sides based on rope width
	// A 0.1 thickness rope will have 6 sides
	// A 1.0 thickness rope will have 12 sides
	float lod_factor = _lod_factor(); // calucate the lod scaling factor
	int min_sides = MIN(get_rope_sides(), 6); // min LOD sizing to 6 sides
	int sides = CLAMP(get_rope_sides() * lod_factor, min_sides, get_rope_sides());

	// emit geometry
	const int last_frame = _frames.size() - 1;

	// move the link instances if present
	bool has_chain = _appearance.is_valid() && _appearance->get_array_mesh().is_valid();
	if (has_chain) {
		auto rs = RenderingServer::get_singleton();
		auto ps = PhysicsServer3D::get_singleton();
		ERR_FAIL_NULL_MSG(rs, "RenderingServer missing");

		bool is_editor = Engine::get_singleton()->is_editor_hint();

		// N-1 chain links per particles
		Vector3 scale_vec(diameter, diameter, diameter);
		for (auto &link : _links) {
			Transform3D xform = link.xform;

#if ENABLE_COLLISION_LAYER
			// physics does not run in editor.
			if (!is_editor && link.physics_body.is_valid()) {
				xform = ps->body_get_state(link.physics_body, PhysicsServer3D::BODY_STATE_TRANSFORM);
			}
#endif

			// xform.orthonormalize();
			if (link.mesh_instance.is_valid()) {
				xform.scale_basis(scale_vec);
				rs->instance_set_transform(link.mesh_instance, xform);
			}
		}

		// clear mesh if previous we weren't rendering chain links
		if (_rope_mesh.is_valid()) {
			_rope_mesh->clear_mesh();
		}
	} else if (_rope_mesh.is_valid()) {
		_rope_mesh->set_radius(radius);
		_rope_mesh->set_sides(sides);
		_rope_mesh->set_rope_length(get_rope_length());
		_rope_mesh->set_rope_width(get_rope_width());
		_rope_mesh->set_rope_twist(get_rope_twist());

		// rebuild
		_rope_mesh->_update_mesh_internal(_frames, get_material());
	}

	// attachments — route through virtual methods so subclasses can override
	if (_appearance.is_valid()) {
		const int att_count = _appearance->get_attachment_count();
		if (att_count > 0) {
			// Default path: use appearance distribution to compute frame index.
			const RopeAppearance::Distribution att_dist = (RopeAppearance::Distribution)_appearance->get_attachment_distribution();
			float cumulative_offset = 0.0f;

			for (int idx = 0; idx < att_count; idx++) {
				const float offset = _appearance->get_attachment_offset(idx);
				const bool from_end = (_appearance->get_attachment_from(idx) == 1);
				const NodePath node = _appearance->get_attachment_nodepath(idx);
				const Transform3D local_xform = _get_attachment_local_transform(idx);

				int frame_idx = 0;
				switch (att_dist) {
					case RopeAppearance::Distribution::ABSOLUTE:
						frame_idx = _frame_at_offset(_frames, offset, from_end);
						break;
					case RopeAppearance::Distribution::RELATIVE:
						cumulative_offset += offset;
						frame_idx = _frame_at_offset(_frames, cumulative_offset, false);
						break;
					case RopeAppearance::Distribution::SCALAR:
						frame_idx = Math::clamp(int(offset * last_frame), 0, last_frame);
						break;
					case RopeAppearance::Distribution::UNIFORM:
						frame_idx = (att_count > 1) ? (idx * last_frame / (att_count - 1)) : 0;
						break;
					default: // covers REAL and unknown — use absolute offset
						frame_idx = _frame_at_offset(_frames, offset, from_end);
						break;
				}

				Transform3D xform = _frames[frame_idx];
				if (idx == att_count - 1) {
					xform = xform.rotated_local(Vector3(1, 0, 0), Math_PI);
				}

				_align_attachment_node(node, xform * local_xform, 0.0);
			}
		}
	}

	update_gizmos();
}

void Rope::_debug_draw_rope() {
	// draw the _initial_pos of the rope in the editor
	if (Engine::get_singleton()->is_editor_hint()) {
		// draw the initial_pos
		PackedVector3Array line_path;
		for (const auto &pos : _initial_pos) {
			line_path.push_back(to_global(pos));
		}
		auto config = DebugDraw3D::new_scoped_config();
		config->set_thickness(0.01);
		DebugDraw3D::draw_line_path(line_path, Color(0, 1, 1));
	}
}

void Rope::_queue_redraw() {
	_rope_dirty = true;
}

bool Rope::_pop_is_dirty() {
	bool is_dirty = _rope_dirty;
	_rope_dirty = false;
	return is_dirty;
}

#pragma endregion

#pragma region Physics Collision

int Rope::get_collision_layer() const { return _collision_layer; }

void Rope::set_collision_layer(int layer) {
	if (layer != _collision_layer) {
		_collision_layer = layer;

#if ENABLE_COLLISION_LAYER
		auto ps = PhysicsServer3D::get_singleton();
		for (auto &link : _links) {
			if (link.physics_body.is_valid()) {
				ps->body_set_collision_layer(link.physics_body, _collision_layer);
			}
		}
#endif
	}
}

int Rope::get_collision_mask() const { return _collision_mask; }

void Rope::set_collision_mask(int mask) {
	if (mask != _collision_mask) {
		_collision_mask = mask;
		_ray_cast->set_collision_mask(_collision_mask);
		_shape_cast->set_collision_mask(_collision_mask);

#if ENABLE_COLLISION_LAYER
		auto ps = PhysicsServer3D::get_singleton();
		for (auto &link : _links) {
			if (link.physics_body.is_valid()) {
				ps->body_set_collision_mask(link.physics_body, _collision_mask);
			}
		}
#endif
	}
}

// NOTE: We do *not* use the collider shapes for collision detection of the rope
// instead we cast rays in the direction of travel and model the particles
// as points. This has trade offs, but its significantly faster than a pin_joint + capsule chain.
void Rope::_apply_constraints() {
	SCOPED_TIMER(_apply_constraints);

	Ref<World3D> w3d = get_world_3d();
	ERR_FAIL_NULL(w3d);

	auto ps = PhysicsServer3D::get_singleton();
	PhysicsDirectSpaceState3D *dss = ps->space_get_direct_state(w3d->get_space());
	ERR_FAIL_NULL(dss);

	// set up the raycast parameters
	if (_collision_mask) {
		float friction = (1.0 - Math::clamp(get_friction(), 0.0f, 1.0f));
		float radius = get_rope_width() * 0.5f;

		float shape_radius = radius / 2.0f;
		if (_collision_use_shape_cast) {
			_collision_shape->set_radius(shape_radius);
			_shape_cast->set_margin(_collision_margin);
		}

		int anchor_count = get_anchor_count();
		int particle_idx = 0;
		while (VALID_PARTICLE_IDX(particle_idx)) {
			auto &particle = _particles[particle_idx];

			// only move non-fixed particles
			if (particle.is_fixed() == false) {
				Vector3 direction = particle.pos_cur - particle.pos_prev;
				float length = direction.length();
				direction.normalize();

				if (length >= CMP_EPSILON) {
					Vector3 margin_vec = direction * shape_radius;
					Vector3 from = particle.pos_prev - margin_vec;
					Vector3 to = particle.pos_cur;
					Vector3 cast_motion = to - from;
					Dictionary hit_result;

					// use shape cast for collision
					if (_collision_use_shape_cast) {
						Transform3D xform(Basis(), from);
						_shape_cast->set_transform(xform);
						_shape_cast->set_motion(cast_motion);

						// test motion first
						PackedFloat32Array result = dss->cast_motion(_shape_cast);
						float safe = result[0];
						float unsafe = result[1];
						if (unsafe < 1.0 || safe < 1.0) {
							// query contact info at the unsafe position to get the collision normal
							Vector3 unsafe_origin = from + cast_motion * unsafe;
							Transform3D unsafe_xform(xform.basis, unsafe_origin);
							_shape_cast->set_transform(unsafe_xform);
							_shape_cast->set_motion(Vector3());
							hit_result = dss->get_rest_info(_shape_cast);
						}

					} else {
						// use ray cast for collision
						_ray_cast->set_from(from);
						_ray_cast->set_to(to);
						hit_result = dss->intersect_ray(_ray_cast);
					}

					if (!hit_result.is_empty()) {
						Vector3 normal = hit_result["normal"];
						Vector3 contact;

						// slightly different contact point for shape vs ray
						if (_collision_use_shape_cast) {
							contact = hit_result["point"];
							contact += (normal * CMP_EPSILON);
							contact += (normal * shape_radius);
						} else {
							contact = hit_result["position"];
							contact += (normal * CMP_EPSILON);
						}

						// reflect acceleration across the contact normal
						particle.accel = particle.accel.bounce(normal) * friction;
						Vector3 velocity = particle.pos_cur - contact;

						particle.pos_prev = contact;
						particle.pos_cur = contact + (velocity * friction).bounce(normal);

						if (_debug_collision) {
							DebugDraw3D::draw_square(contact, 0.005, Color(1, 1, 0, 0.1));
							DebugDraw3D::draw_sphere(particle.pos_cur, shape_radius, Color(1, 1, 0));
						}
					}
				}
			}

			// advance
			particle_idx++;
		}
	}
}

#pragma endregion

#pragma region Physics Update

void Rope::_update_physics(float delta, int iterations) {
	SCOPED_TIMER(_update_physics);

	// update anchor transforms and behaviors
	_internal_update_anchors();

	for (int i = 0; i < iterations; i++) {
		_apply_forces();

		// turn this off when using physics bodies.
		_apply_constraints();
		_verlet_process(delta);

		_stiff_rope(_stiffness_iterations);
		_balance_tension();

		// now that everything has moved, recalc link positions
		_calculate_links_for_particles();
		_update_links();
	}
}

// Update the particle positions & behaviors based upon the current anchor layout
void Rope::_internal_update_anchors() {
	SCOPED_TIMER(_internal_update_anchors);

	// default all particles to no anchor
	for (Particle &p : _particles) {
		p.anchor_idx = -1;
		p.behavior = FREE;
	}

	// index counters
	int anchor_count = _anchors.size();
	int anchor_idx = 0;
	int particle_idx = 0;
	int anchor_part_idx = 0;
	int anchor_part_count = 0;
	Transform3D xform;
	int behavior = FREE;

	// now fill particles for anchors. if anchors overlap they will span multiple particles.
	while (VALID_ANCHOR_IDX(anchor_idx)) {
		// first in part?
		if (anchor_part_count == 0) {
			anchor_part_count = get_anchor_particle_count(anchor_idx);
			anchor_part_idx = 0;

			// set base particle index from the absolute offset
			float abs_offset = get_anchor_abs_offset(anchor_idx);
			particle_idx = _get_particle_for_offset(abs_offset);

			// update index back in anchor
			_anchors[anchor_idx].particle_idx = particle_idx;
		}

		// update particle if index is valid
		if (VALID_PARTICLE_IDX(particle_idx)) {
			if (anchor_part_idx == 0) {
				xform = get_anchor_transform(anchor_idx);
				behavior = get_anchor_behavior(anchor_idx);
				_particles[particle_idx].anchor_idx = anchor_idx;
			} else {
				_particles[particle_idx].anchor_idx = -1;
			}

			// fix the particle position if not free
			if (behavior != FREE) {
				Vector3 pos = get_anchor_particle_position(anchor_idx, anchor_part_idx);
				_particles[particle_idx].set_position(xform.xform(pos));
			}
			_particles[particle_idx].behavior = behavior;
		}

		// move to the next particle and anchor
		particle_idx++;
		anchor_part_idx++;
		if (anchor_part_idx >= anchor_part_count) {
			anchor_idx++;

			// reset anchor part for next anchor
			anchor_part_count = 0;
			anchor_part_idx = 0;
		}
	}
}

void Rope::_prepare_physics_server() {
	// no-op for now
}

void Rope::_apply_anchor_forces(Particle &p_particle, int p_anchor_idx, const Vector3 &tension, bool draw_debug) {
	// apply reaction force to p0's attached anchor (tension pulls it toward p1)
	if (_is_anchor_moving(p_anchor_idx, _anchors.size()) && _anchors[p_anchor_idx].rigid_body_id != 0) {
		// Rope pulls on the rigid body associated with the anchor.
		RigidBody3D *rigid_body = get_anchor_rigidbody(p_anchor_idx);
		if (rigid_body != nullptr && _max_tension_force > 0.0f) {
			Vector3 applied_force = tension * rigid_body->get_mass();
			float force_magnitude = applied_force.length();
			if (force_magnitude > _max_tension_force) {
				applied_force = applied_force.normalized() * _max_tension_force;
			}

			// apply force at anchor position
			// NOTE: is pos_cur in global space? do transform if necessary
			rigid_body->apply_force(applied_force, p_particle.pos_cur - rigid_body->get_global_position());

			// draw tension
			if (draw_debug && _debug) {
				Vector3 global_pos = p_particle.pos_cur;

				float length = applied_force.length() / _max_tension_force;
				Color low(0, 0, 1), high(1, 0, 0);
				Color color = low.lerp(high, length);
				DebugDraw3D::draw_arrow_ray(global_pos, applied_force.normalized(), 0.5 + length, color, 0.1f);

				String label = String::num(Math::snapped(tension.length(), 0.01));
				DebugDraw3D::draw_text(global_pos + Vector3(0, 0.25, 0), label, 16, color);
			}
		}
	}
}

void Rope::_stiff_rope(int iterations) {
	SCOPED_TIMER(_stiff_rope);

	// calculate the maximum allowed rope length
	const float max_segment_length = _get_average_segment_length();
	const float stiffness = Math::clamp(_stiffness, 0.0f, 2.0f);

	int anchor_count = get_anchor_count();

	// Position relaxation iterations — correct particle positions toward rest length.
	for (int j = 0; j < iterations; j++) {
		Vector3 tension_direction_sum = Vector3();

		int particle_idx = 0;
		while (particle_idx >= 0 && particle_idx < _particles.size() - 1) {
			Particle &p0 = _particles[particle_idx];
			Particle &p1 = _particles[particle_idx + 1];

			const Vector3 segment = p1.pos_cur - p0.pos_cur;
			const Vector3 direction = segment.normalized();
			const float length = segment.length();
			float stretch = (length - max_segment_length);

			// calculate tension before clamping
			// stretch is scaled by stiffness and force scale
			Vector3 tension = direction * stretch * stiffness * _tension_force_scale;

			// prevent overshoot
			stretch = Math::clamp(stretch * stiffness, -length, length);
			p0.stretch = stretch;

			// only anchored and guided anchors stay put
			bool anchored_0 = !p0.is_free();
			bool anchored_1 = !p1.is_free();

			// only debug_draw last iteration
			bool draw_debug = (j == iterations - 1) && _debug;

			// if both are attached skip
			if (anchored_0 && anchored_1) {
				// no-op

				// if either particle is attached, only move the other one
			} else if (anchored_0) {
				p1.pos_cur -= direction * stretch;
				_apply_anchor_forces(p0, p0.anchor_idx, tension, draw_debug);

			} else if (anchored_1) {
				p0.pos_cur += direction * stretch;
				_apply_anchor_forces(p1, p1.anchor_idx, -tension, draw_debug);

				// neither are attached, half stretch
			} else {
				const Vector3 half_stretch = direction * 0.5f * stretch;
				p0.pos_cur += half_stretch;
				p1.pos_cur -= half_stretch;
			}

			particle_idx++;
		}
	}
}

// Balance tension moves the index of the anchor up and down the rope to equalize tension
// for GUIDED and SLIDING anchors. Indexes will not move past an existing index.
void Rope::_balance_tension() {
	SCOPED_TIMER(_balance_tension);

	int anchor_count = get_anchor_count();

	// need a minimum of 3 particles and 3 anchors to balance tension
	if (_particles.size() < 3 || anchor_count < 3) {
		return;
	}

	// iterate over mid-anchors only (skip start and end)
	float segment_length = _get_average_segment_length();
	float tolerance = segment_length * 0.25f; // NOTE: what to set this to? This seems mostly okay...
	for (uint64_t anchor_idx = 1; anchor_idx < anchor_count - 1; anchor_idx++) {
		// only balance GUIDED and SLIDING anchors — these allow the rope to slide through
		if (!_is_rope_sliding(anchor_idx, anchor_count)) {
			continue;
		}

		// particle for anchor out of range, skip
		int64_t idx = _anchors[anchor_idx].particle_idx;
		if (idx < 1 || idx >= (int64_t)_particles.size() - 1) {
			continue;
		}

		// stretch of the segment to the left (idx-1 -> idx) is stored on _particles[idx-1]
		// stretch of the segment to the right (idx -> idx+1) is stored on _particles[idx]

		// get previous and next anchor offsets
		float prev_width = segment_length * (get_anchor_particle_count(anchor_idx - 1) + 1);
		float prev_offset = get_anchor_abs_offset(anchor_idx - 1) + prev_width;

		float next_width = segment_length * (get_anchor_particle_count(anchor_idx + 1) + 1);
		float next_offset = get_anchor_abs_offset(anchor_idx + 1) - next_width;

		// this will clamp to the ends for offset overuns
		int prev_idx = _get_particle_for_offset(prev_offset);
		int next_idx = _get_particle_for_offset(next_offset);

		float left_stretch = _particles[prev_idx].stretch;
		float right_stretch = _particles[next_idx].stretch;

		float stretch_diff = Math::abs(left_stretch - right_stretch);

		// require the imbalance to exceed a fraction of segment length before shifting,
		// this prevents oscillation when the two sides are nearly balanced.
		if (stretch_diff < tolerance) {
			continue;
		}

		// move the abs_offset in the direction of lower stretch
		float tension_speed = segment_length * (1.0 - get_anchor_friction(anchor_idx));
		float direction = (left_stretch > right_stretch) ? 1.0f : -1.0f;
		float offset_change = direction * tension_speed;
		float cur_offset = get_anchor_abs_offset(anchor_idx);

		float new_offset = Math::clamp(cur_offset + offset_change, prev_offset, next_offset);

		// ...and slide the anchor offset. particle_idx will get update on next _internal_update_anchors call
		_anchors[anchor_idx]._abs_offset = new_offset;
	}
}

void Rope::_verlet_process(float delta) {
	SCOPED_TIMER(_verlet_process);
	int anchor_count = get_anchor_count();

	int particle_idx = 0;
	while (VALID_PARTICLE_IDX(particle_idx)) {
		auto &particle = _particles[particle_idx];

		// is this point not fixed in space?
		if (particle.is_fixed() == false) {
			Vector3 position_current_copy = particle.pos_cur;
			particle.pos_cur = (2.0 * particle.pos_cur) - particle.pos_prev + (delta * delta * particle.accel);
			particle.pos_prev = position_current_copy;
		}

		particle_idx++;
	}
}

void Rope::_apply_forces() {
	SCOPED_TIMER(_apply_forces);

	// cylinder volume
	float rope_volume = Math_PI * Math::pow(get_rope_width() * 0.5f, 2) * get_rope_length();
	if (rope_volume <= 0.0f)
		rope_volume = 1.0f;

	// NOTE: becuase we don't have mass in verlet, we simulate buoyancy with a ratio.
	// 1.0 == same as water, 0.5 == sink, 1.5 == floaty

	// each point accounts for a portion of the rope volume
	float probe_volume = 0.0f;
	if (_particles.size() > 0) {
		probe_volume = rope_volume / _particles.size();
	}

	// Pre-calculate constants used in the loop
	float rope_width = get_rope_width();
	float rope_radius = rope_width * 0.5f;
	Vector3 gravity_scaled = _gravity * _gravity_scale;
	float pi_r_squared = Math_PI * rope_radius * rope_radius;
	LiquidArea *liquid_area = get_liquid_area();

	int anchor_count = get_anchor_count();

	int particle_idx = 0;
	while (VALID_PARTICLE_IDX(particle_idx)) {
		auto &particle = _particles[particle_idx];

		// forces act only on unattached
		if (particle.is_fixed() == false) {
			float submerged_ratio = 0.0f;
			Vector3 total_acceleration = Vector3(0, 0, 0);
			Vector3 velocity = particle.pos_cur - particle.pos_prev;

			if (_apply_gravity) {
				total_acceleration += _gravity * _gravity_scale;
			}

			// Because gravity is "unit mass" in verlet, we simulate buoyancy here
			// based on the submerged volume of the rope segment with buoyancy forces as a ratio.
			if (_apply_buoyancy && liquid_area) {
				Vector3 probe = particle.pos_cur;

				// Get wave transform at this position
				Transform3D wave_xform = Transform3D(Basis(), probe);
				wave_xform = liquid_area->get_liquid_transform(probe);

				// Calculate depths.
				float wave_depth = probe.y - wave_xform.origin.y;

				// calculate the submerged volume for a cylinder section.
				// when depth == 0, rope is half submerged.
				float h = Math::clamp(-wave_depth, 0.0f, get_rope_width());
				float r = get_rope_width() * 0.5f;
				float submerged_area = r * r * Math::acos((r - h) / r) - (r - h) * Math::sqrt(2 * r * h - h * h);
				float submerged_volume = submerged_area * (probe_volume / (Math_PI * r * r));
				submerged_ratio = submerged_volume / probe_volume;

				// If submerged (depth < 0), apply buoyancy force
				if (submerged_ratio > 0.0f) {
					// this is how floaty the rope is...
					float buoyancy_factor = _buoyancy_scale; //liquid_density / _rope_density; ... could also do it this way

					// lerp the wave normal with the up vector based on buoyancy factor
					// this makes the rope float along the surface when partially submerged
					// and float straight up when fully submerged.
					Vector3 wave_normal = wave_xform.basis.get_column(1).normalized();
					wave_normal = wave_normal.slerp(Vector3(0, 1, 0), Math::clamp(1.0f - buoyancy_factor, 0.0f, 1.0f));

					Vector3 wave_force = wave_normal * _gravity * Math::clamp(wave_depth, -1.0f, 0.0f) * buoyancy_factor * submerged_ratio;

					// Apply buoyancy force
					total_acceleration += wave_force;

					// Current and drag forces act on velocity relative to water velocity
					// F_drag = drag_coefficient * (current_velocity - velocity)
					//        = drag_coefficient * current_velocity - drag_coefficient * velocity
					// First term: water current pushes the object
					// Second term: drag slows down motion through water
					Vector3 current_velocity = liquid_area->get_liquid_velocity();
					Vector3 current_force = current_velocity * CLAMP(_submerged_drag, 0.0, 10.0) * submerged_ratio; // clamped drag range. this is a fudge at the moment because we're not using real mass properties.
					Vector3 drag_force = -velocity * _submerged_drag * submerged_ratio;

					total_acceleration += current_force;
					total_acceleration += drag_force;
				}
			}

			// Wind does not apply when submerged
			if (_apply_wind && _wind_noise != nullptr && submerged_ratio == 0.0f) {
				Vector3 timed_position = particle.pos_cur + Vector3(1, 1, 1) * _time;
				float wind_force = _wind_noise->get_noise_3d(timed_position.x, timed_position.y, timed_position.z);
				total_acceleration += _wind_scale * _wind * wind_force;
			}

			// Additional damping
			if (_apply_damping) {
				_damping_factor = Math::clamp(_damping_factor, 0.0f, 10000.0f);
				Vector3 drag = -_damping_factor * velocity.length() * velocity;
				total_acceleration += drag;
			}

			particle.accel = total_acceleration;
		}

		// advance
		particle_idx++;
	}
}

#pragma endregion