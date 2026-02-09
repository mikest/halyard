/* Ocean

A class that provides ocean wave simulation with height displacement based on a noise texture.
*/
#include "ocean_area.h"

#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/classes/viewport.hpp>
#include <godot_cpp/classes/viewport_texture.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/editor_interface.hpp>
#include <godot_cpp/classes/sub_viewport.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/variant/callable.hpp>

using namespace godot;

OceanArea::OceanArea() {
}

OceanArea::~OceanArea() {
}

OceanArea *OceanArea::get_ocean_area() {
    SceneTree *p_tree = Object::cast_to<SceneTree>(Engine::get_singleton()->get_main_loop());

    if (!p_tree) {
        return nullptr;
    }
    
    // Search for first Ocean node in the scene
    Node *root = p_tree->get_current_scene();
    if (!root) {
        return nullptr;
    }
    
    // Try to find by unique name first (common pattern)
    Node *ocean_node = root->get_node_or_null(NodePath("%Ocean"));
    if (ocean_node) {
        OceanArea *ocean = Object::cast_to<OceanArea>(ocean_node);
        if (ocean) {
            return ocean;
        }
    }
    
    // Fallback: search tree for any Ocean instance
    TypedArray<Node> nodes = root->find_children("*", "Ocean", true, false);
    if (nodes.size() > 0) {
        return Object::cast_to<OceanArea>(static_cast<Object *>(nodes[0]));
    }
    
    return nullptr;
}

void OceanArea::_notification(int p_what) {
    // give wave sampler a chance to init before we try and use it.
    if (_wave_sampler.is_valid()) {
       _wave_sampler->_notification(p_what);
    }

    RenderingServer *rs = RenderingServer::get_singleton();
    ERR_FAIL_COND_MSG(!rs, "RenderingServer not available");

    switch (p_what) {
        case NOTIFICATION_ENTER_TREE:
            if (rs->is_connected("frame_post_draw", Callable(this, "_frame_post_draw")) == false) {
                rs->connect("frame_post_draw", Callable(this, "_frame_post_draw"));
             }
            break;
        case NOTIFICATION_READY:
            set_process(true);

            // Try to find a child clipmap instance in the scene if not already assigned
            if (_clipmap_instance == nullptr) {
                _clipmap_instance = Object::cast_to<ClipMapInstance>(get_node_or_null("ClipMapInstance"));
            }
            if (_ocean_detailer == nullptr) {
                _ocean_detailer = Object::cast_to<OceanDetailer>(get_node_or_null("OceanDetailer"));
            }
            if (_wave_sampler.is_valid()) {
                _wave_sampler->set_detail_map(_ocean_detailer ? _ocean_detailer->get_texture() : Ref<Texture2D>());
                _wave_sampler->refresh_from_material();
            }
            break;
        case NOTIFICATION_PROCESS:
            // Update shader parameter for vertex scaling
            if (_wave_sampler.is_valid()) {
                Ref<ShaderMaterial> material = _wave_sampler->get_material();
                if (material.is_valid()) {

                    material->set_shader_parameter("VertexScaling", _vertex_scaling);
                    _wave_sampler->set_wave_time(static_cast<float>(Time::get_singleton()->get_ticks_msec()) / 1000.0f);

                    // ferry the detailer properties over to the sampler
                    _wave_sampler->set_details_texture_size(_ocean_detailer ? _ocean_detailer->get_detail_viewport_texture_size() : 128.0f);
                    
                    _wave_sampler->set_details_corner_position(_ocean_detailer ? _ocean_detailer->get_detail_viewport_texture_corner_position() : Vector3(0,0,0));

                    _wave_sampler->set_details_vertex_spacing(_ocean_detailer ? _ocean_detailer->get_detail_viewport_vertex_spacing() : 1.0f);
                }
    
            }   
            // Update clipmap scaling
            if (_clipmap_instance) {
                _clipmap_instance->set_vertex_scaling(_vertex_scaling);
            }
            _update_clipmap_position();
            break;
        case NOTIFICATION_EXIT_TREE:
            if (rs->is_connected("frame_post_draw", Callable(this, "_frame_post_draw"))) {
                rs->disconnect("frame_post_draw", Callable(this, "_frame_post_draw"));
            }
            break;
    }
}

void OceanArea::set_wave_sampler(const Ref<WaveSampler> &p_sampler) {
    _wave_sampler = p_sampler;
}

Ref<WaveSampler> OceanArea::get_wave_sampler() const {
    return _wave_sampler;
}

void OceanArea::set_clipmap_instance(ClipMapInstance *p_instance) {
    _clipmap_instance = p_instance;
    if (_clipmap_instance) {
        _clipmap_instance->set_vertex_scaling(_vertex_scaling);
    }
}

ClipMapInstance *OceanArea::get_clipmap_instance() const {
    return _clipmap_instance;
}


void OceanArea::set_ocean_detailer(OceanDetailer *p_instance) {
    _ocean_detailer = p_instance;
}


OceanDetailer *OceanArea::get_ocean_detailer() const {
    return _ocean_detailer;
}

void OceanArea::set_ocean_update(OceanUpdate p_mode) {
    _ocean_update = p_mode;
}

OceanArea::OceanUpdate OceanArea::get_ocean_update() const {
    return _ocean_update;
}


void OceanArea::set_vertex_scaling(float p_scale) {
    _vertex_scaling = p_scale;
    if (_clipmap_instance) {
        _clipmap_instance->set_vertex_scaling(_vertex_scaling);
    }
}

float OceanArea::get_vertex_scaling() const {
    return _vertex_scaling;
}

void OceanArea::set_tidal_override(float p_override) {
    _tidal_override = p_override;
}

float OceanArea::get_tidal_override() const {
    return _tidal_override;
}

void OceanArea::set_low_tide(float p_low) {
    _low_tide = p_low;
}

float OceanArea::get_low_tide() const {
    return _low_tide;
}

void OceanArea::set_high_tide(float p_high) {
    _high_tide = p_high;
}

float OceanArea::get_high_tide() const {
    return _high_tide;
}

void OceanArea::set_tide_phase(float p_phase) {
    _tide_phase = CLAMP(p_phase, 0.0f, 1.0f);
}

float OceanArea::get_tide_phase() const {
    return _tide_phase;
}

float OceanArea::get_tide_height() const {
    float tide_value = _tide_phase;
    if (_tidal_override >= 0.0f) {
        tide_value = _tidal_override;
    }
    return Math::lerp(_low_tide, _high_tide, tide_value);
}

void OceanArea::_update_clipmap_position() {
    if (!_clipmap_instance || _ocean_update == DISABLED || !is_inside_tree()) {
        return;
    }

    Vector3 world_pos = get_global_position();
    if (_ocean_update == CAMERA) {
        Camera3D *camera = nullptr;
        Viewport *viewport = nullptr;
        
        // In editor, use editor viewport camera
        if (Engine::get_singleton()->is_editor_hint()) {
            viewport = EditorInterface::get_singleton()->get_editor_viewport_3d();
        } else {
            // Runtime: use viewport camera
            viewport = get_viewport();
        }
        
        // update tracking coordinates for clipmap and detailer.
        camera = viewport ? viewport->get_camera_3d() : nullptr;
        world_pos = camera ? camera->get_global_position() : get_global_position();
    }

    if (_clipmap_instance) {
        // tidal height is controlled by the clipmap global position.
        world_pos.y = get_tide_height();
        _clipmap_instance->update_position(world_pos);
    }
}

float OceanArea::_get_current_time() const {
    return static_cast<float>(Time::get_singleton()->get_ticks_msec()) / 1000.0f;
}

Transform3D OceanArea::_get_wave_transform_at_position(const Vector3 &pos, float time) const {
    ERR_FAIL_COND_V_MSG(!_clipmap_instance, Transform3D(), "ClipMapInstance not assigned");

    Transform3D global_xform = _clipmap_instance->get_global_transform();
    float base_y = global_xform.origin.y;

    // If no wave sampler or invalid, return flat surface at tide height
    if (_wave_sampler->is_valid()== false) {
        Transform3D result;
        result.origin = Vector3(pos.x, base_y, pos.z);
        return result;
    }

    // Set wave time before sampling
    _wave_sampler->set_wave_time(time);
    
    return _wave_sampler->get_wave_transform(pos, base_y, global_xform);
}



void OceanArea::_internal_update_transforms_for_points(const PackedVector3Array &global_points, TypedArray<Transform3D> r_transforms) const {
    int point_count = global_points.size();
    r_transforms.resize(point_count);

    float time = _get_current_time();
    for (int i = 0; i < point_count; i++) {
        r_transforms[i] = _get_wave_transform_at_position(global_points[i], time);
    }
}

void OceanArea::_frame_post_draw() {
    // After a frame has rendered, fetch shader parameters and textures
    if (_wave_sampler.is_valid()) {
        _wave_sampler->refresh_from_material();
    }
}


void OceanArea::_bind_methods() {
    // Wave sampler
    ClassDB::bind_method(D_METHOD("set_wave_sampler", "sampler"), &OceanArea::set_wave_sampler);
    ClassDB::bind_method(D_METHOD("get_wave_sampler"), &OceanArea::get_wave_sampler);
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "wave_sampler", PROPERTY_HINT_RESOURCE_TYPE, "WaveSampler"), "set_wave_sampler", "get_wave_sampler");

    // Ocean update mode
    ClassDB::bind_method(D_METHOD("set_ocean_update", "mode"), &OceanArea::set_ocean_update);
    ClassDB::bind_method(D_METHOD("get_ocean_update"), &OceanArea::get_ocean_update);
    ADD_PROPERTY(PropertyInfo(Variant::INT, "ocean_update", PROPERTY_HINT_ENUM, "Disabled,Camera"), "set_ocean_update", "get_ocean_update");
    
    ClassDB::bind_method(D_METHOD("set_vertex_scaling", "scale"), &OceanArea::set_vertex_scaling);
    ClassDB::bind_method(D_METHOD("get_vertex_scaling"), &OceanArea::get_vertex_scaling);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "vertex_scaling", PROPERTY_HINT_RANGE, "0.03125,32,0.03125"), "set_vertex_scaling", "get_vertex_scaling");

    // Clipmap instance access
    ClassDB::bind_method(D_METHOD("set_clipmap_instance", "instance"), &OceanArea::set_clipmap_instance);
    ClassDB::bind_method(D_METHOD("get_clipmap_instance"), &OceanArea::get_clipmap_instance);
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "clipmap_instance", PROPERTY_HINT_NODE_TYPE, "ClipMapInstance"), "set_clipmap_instance", "get_clipmap_instance");

    // Ocean detailer access
    ClassDB::bind_method(D_METHOD("set_ocean_detailer", "instance"), &OceanArea::set_ocean_detailer);
    ClassDB::bind_method(D_METHOD("get_ocean_detailer"), &OceanArea::get_ocean_detailer);
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "ocean_detailer", PROPERTY_HINT_NODE_TYPE, "OceanDetailer"), "set_ocean_detailer", "get_ocean_detailer");

    // Tide properties
    ClassDB::bind_method(D_METHOD("set_tidal_override", "override"), &OceanArea::set_tidal_override);
    ClassDB::bind_method(D_METHOD("get_tidal_override"), &OceanArea::get_tidal_override);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "tidal_override", PROPERTY_HINT_RANGE, "-1,1,0.01,or_greater"), "set_tidal_override", "get_tidal_override");

    ClassDB::bind_method(D_METHOD("set_low_tide", "low_tide"), &OceanArea::set_low_tide);
    ClassDB::bind_method(D_METHOD("get_low_tide"), &OceanArea::get_low_tide);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "low_tide", PROPERTY_HINT_RANGE, "-100,100,0.1"), "set_low_tide", "get_low_tide");

    ClassDB::bind_method(D_METHOD("set_high_tide", "high_tide"), &OceanArea::set_high_tide);
    ClassDB::bind_method(D_METHOD("get_high_tide"), &OceanArea::get_high_tide);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "high_tide", PROPERTY_HINT_RANGE, "-100,100,0.1"), "set_high_tide", "get_high_tide");

    ClassDB::bind_method(D_METHOD("set_tide_phase", "phase"), &OceanArea::set_tide_phase);
    ClassDB::bind_method(D_METHOD("get_tide_phase"), &OceanArea::get_tide_phase);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "tide_phase", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_tide_phase", "get_tide_phase");

    ClassDB::bind_method(D_METHOD("get_tide_height"), &OceanArea::get_tide_height);

    // Internal callback method
    ClassDB::bind_method(D_METHOD("_frame_post_draw"), &OceanArea::_frame_post_draw);

    // Static method
    ClassDB::bind_static_method("OceanArea", D_METHOD("get_ocean_area"), &OceanArea::get_ocean_area);

    // Enum
    BIND_ENUM_CONSTANT(DISABLED);
    BIND_ENUM_CONSTANT(CAMERA);
}