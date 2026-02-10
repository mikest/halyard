/* OceanDetailer Implementation */

#include "ocean_detailer.h"

#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/camera_attributes.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/environment.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/classes/viewport.hpp>
#include <godot_cpp/classes/viewport_texture.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

OceanDetailer::OceanDetailer() {
	_global_shader_init();
	set_process(true);
}

OceanDetailer::~OceanDetailer() {
	// Cleanup camera if it exists
	if (_camera != nullptr && _camera->is_inside_tree()) {
		remove_child(_camera);
		memdelete(_camera);
		_camera = nullptr;
	}
}

void OceanDetailer::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {
			_global_shader_init();
		} break;

		case NOTIFICATION_READY: {
			// Apply render size to viewport
			set_size(Vector2i(_render_size * 4, _render_size * 4));

			// Create and add camera as child at runtime
			if (_camera == nullptr) {
				_camera = memnew(Camera3D);
				_camera->set_name("OceanDetailerCamera");
				_camera->make_current();
				add_child(_camera, true);
				_camera->set_owner(this);
			}

			if (_camera) {
				// Create an Environment and set it
				Ref<Environment> environment;
				environment.instantiate();
				environment->set_background(Environment::BG_COLOR);
				environment->set_bg_color(Color(0, 0, 0, 0.0)); // transparent
				environment->set_ambient_source(Environment::AMBIENT_SOURCE_DISABLED);
				environment->set_reflection_source(Environment::REFLECTION_SOURCE_DISABLED);
				_camera->set_environment(environment);

				// Configure camera settings for ocean detail rendering
				_camera->set_projection(Camera3D::PROJECTION_ORTHOGONAL);
				_camera->set_size(_render_size);
				_camera->set_near(0.1f);
				_camera->set_far(4000.0f);

				// Position camera above plane looking down
				_camera->set_rotation(Vector3(-Math_PI / 2.0f, 0, 0));
				_camera->set_position(Vector3(0, 1000, 0));

				// Set the cull mask to only include channel 19, our wave rendering channel
				_camera->set_cull_mask(WAVE_CULL_MASK);
			}

			// enable transparency and HDR2 so we can have wave hieghts > 1.0
			set_transparent_background(true);
			set_use_hdr_2d(true);

			RenderingServer *rs = RenderingServer::get_singleton();
			if (rs) {
				rs->global_shader_parameter_set("DetailViewportTexture", get_texture());
			}
			_global_shader_update();
		} break;

		case NOTIFICATION_PROCESS: {
			// Update position based on follow target
			if (_follow_target != nullptr && _follow_target->is_inside_tree()) {
				Vector3 target_pos = _follow_target->get_global_position();
				float distance = _last_follow_position.distance_to(target_pos);

				if (distance >= _snap_step) {
					update_position(target_pos);
					_last_follow_position = target_pos;
					_global_shader_update();
				}
			}
		} break;

		case NOTIFICATION_EXIT_TREE: {
		} break;
	}
}

#pragma region Property Setters/Getters

void OceanDetailer::set_render_size(float p_size) {
	_render_size = p_size;
	if (is_inside_tree()) {
		set_size(Vector2i(_render_size, _render_size));
	}
}

float OceanDetailer::get_render_size() const {
	return _render_size;
}

void OceanDetailer::set_reposition_rate(float p_rate) {
	_reposition_rate = p_rate;
}

float OceanDetailer::get_reposition_rate() const {
	return _reposition_rate;
}

void OceanDetailer::set_snap_step(float p_step) {
	_snap_step = p_step;
}

float OceanDetailer::get_snap_step() const {
	return _snap_step;
}

void OceanDetailer::set_follow_target(Node3D *p_target) {
	_follow_target = p_target;
	if (_follow_target != nullptr) {
		_last_follow_position = _follow_target->get_global_position();
	}
}

Node3D *OceanDetailer::get_follow_target() const {
	return _follow_target;
}

Camera3D *OceanDetailer::get_camera() const {
	return _camera;
}

void OceanDetailer::update_position(const Vector3 &p_global_position) {
	if (_camera == nullptr) {
		return;
	}

	// Snap position to grid based on snap_step
	Vector3 snapped_pos = p_global_position;
	snapped_pos.x = Math::snapped(snapped_pos.x, _snap_step);
	snapped_pos.z = Math::snapped(snapped_pos.z, _snap_step);

	// Keep Y at fixed height (1000 units above)
	snapped_pos.y = 1000.0f;

	_camera->set_global_position(snapped_pos);
}

Vector3 OceanDetailer::get_snapped_position() const {
	if (_camera == nullptr) {
		return Vector3();
	}

	Vector3 pos = _camera->get_global_position();
	return pos;
}

void OceanDetailer::_global_shader_update() {
	RenderingServer *rs = RenderingServer::get_singleton();
	ERR_FAIL_COND_MSG(rs == nullptr, "RenderingServer singleton not found. Cannot initialize global shader parameters for OceanDetailer.");

	ERR_FAIL_COND_MSG(get_texture() == nullptr, "DetailViewportTexture is null. Cannot update global shader parameters for OceanDetailer.");

	// Initialize global shader parameters used by Ocean shaders
	// These parameters allow the ocean shader to sample from this detail viewport
	rs->global_shader_parameter_set("DetailViewportTextureSize", get_detail_viewport_texture_size());
	rs->global_shader_parameter_set("DetailViewportTextureCornerPosition", get_detail_viewport_texture_corner_position());
	rs->global_shader_parameter_set("DetailViewportVertexSpacing", get_detail_viewport_vertex_spacing());
}

float OceanDetailer::get_detail_viewport_texture_size() const {
	return _render_size;
}

Vector3 OceanDetailer::get_detail_viewport_texture_corner_position() const {
	Vector3 offset = Vector3(0, 0, 0);

	if (_camera) {
		offset = _camera->get_global_position() - Vector3(_render_size / 2.0f, 0, _render_size / 2.0f);
	}

	return offset;
}

float OceanDetailer::get_detail_viewport_vertex_spacing() const {
	if (get_size().x > 0) {
		return _render_size / static_cast<float>(get_size().x);
	}
	return 0.0f;
}

#pragma endregion

void OceanDetailer::_global_shader_init() {
	// Only initialize global shader parameters in the editor
	if (Engine::get_singleton()->is_editor_hint()) {
		// create global shader parameters in engine if they don't exist yet
		RenderingServer *rs = RenderingServer::get_singleton();
		if (rs) {
			TypedArray<StringName> existing_params = rs->global_shader_parameter_get_list();

			if (!existing_params.has("DetailViewportTexture")) {
				rs->global_shader_parameter_add("DetailViewportTexture", RenderingServer::GLOBAL_VAR_TYPE_SAMPLER2D, nullptr);
			}
			if (!existing_params.has("DetailViewportTextureSize")) {
				rs->global_shader_parameter_add("DetailViewportTextureSize", RenderingServer::GLOBAL_VAR_TYPE_FLOAT, 0.0f);
			}
			if (!existing_params.has("DetailViewportTextureCornerPosition")) {
				rs->global_shader_parameter_add("DetailViewportTextureCornerPosition", RenderingServer::GLOBAL_VAR_TYPE_VEC3, Vector3());
			}
			if (!existing_params.has("DetailViewportVertexSpacing")) {
				rs->global_shader_parameter_add("DetailViewportVertexSpacing", RenderingServer::GLOBAL_VAR_TYPE_FLOAT, 0.0f);
			}
		}
	}
}

void OceanDetailer::_bind_methods() {
	// Property bindings
	ClassDB::bind_method(D_METHOD("set_render_size", "size"), &OceanDetailer::set_render_size);
	ClassDB::bind_method(D_METHOD("get_render_size"), &OceanDetailer::get_render_size);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "render_size", PROPERTY_HINT_RANGE, "32.0,4096.0,1.0"), "set_render_size", "get_render_size");

	ClassDB::bind_method(D_METHOD("set_reposition_rate", "rate"), &OceanDetailer::set_reposition_rate);
	ClassDB::bind_method(D_METHOD("get_reposition_rate"), &OceanDetailer::get_reposition_rate);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "reposition_rate", PROPERTY_HINT_RANGE, "0.0,10.0,0.01"), "set_reposition_rate", "get_reposition_rate");

	ClassDB::bind_method(D_METHOD("set_snap_step", "step"), &OceanDetailer::set_snap_step);
	ClassDB::bind_method(D_METHOD("get_snap_step"), &OceanDetailer::get_snap_step);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "snap_step", PROPERTY_HINT_RANGE, "0.1,100.0,0.1"), "set_snap_step", "get_snap_step");

	ClassDB::bind_method(D_METHOD("set_follow_target", "target"), &OceanDetailer::set_follow_target);
	ClassDB::bind_method(D_METHOD("get_follow_target"), &OceanDetailer::get_follow_target);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "follow_target", PROPERTY_HINT_NODE_TYPE, "Node3D"), "set_follow_target", "get_follow_target");

	ClassDB::bind_method(D_METHOD("get_detail_viewport_texture_size"), &OceanDetailer::get_detail_viewport_texture_size);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "detail_viewport_texture_size", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR | PROPERTY_USAGE_READ_ONLY), "", "get_detail_viewport_texture_size");

	ClassDB::bind_method(D_METHOD("get_detail_viewport_texture_corner_position"), &OceanDetailer::get_detail_viewport_texture_corner_position);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "detail_viewport_texture_corner_position", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR | PROPERTY_USAGE_READ_ONLY), "", "get_detail_viewport_texture_corner_position");

	ClassDB::bind_method(D_METHOD("get_detail_viewport_vertex_spacing"), &OceanDetailer::get_detail_viewport_vertex_spacing);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "detail_viewport_vertex_spacing", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR | PROPERTY_USAGE_READ_ONLY), "", "get_detail_viewport_vertex_spacing");

	// Camera accessor
	ClassDB::bind_method(D_METHOD("get_camera"), &OceanDetailer::get_camera);

	// Position update
	ClassDB::bind_method(D_METHOD("update_position", "global_position"), &OceanDetailer::update_position);
	ClassDB::bind_method(D_METHOD("get_snapped_position"), &OceanDetailer::get_snapped_position);

	// Constants
	BIND_CONSTANT(WAVE_CULL_MASK);
}