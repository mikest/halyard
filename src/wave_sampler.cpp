/* WaveSampler

A Resource class that samples wave height and normal from a shader material's parameters.
*/
#include "wave_sampler.h"

#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/classes/rendering_device.hpp>
#include <godot_cpp/variant/callable.hpp>
#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/texture2d.hpp>
#include <godot_cpp/classes/image.hpp>

using namespace godot;

WaveSampler::WaveSampler() {
}

WaveSampler::~WaveSampler() {
}

void WaveSampler::_bind_methods() {
    // Material
    ClassDB::bind_method(D_METHOD("set_material", "material"), &WaveSampler::set_material);
    ClassDB::bind_method(D_METHOD("get_material"), &WaveSampler::get_material);
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "material", PROPERTY_HINT_RESOURCE_TYPE, "ShaderMaterial"), "set_material", "get_material");

    // Height and detail map textures
    ClassDB::bind_method(D_METHOD("set_height_map", "texture"), &WaveSampler::set_height_map);
    ClassDB::bind_method(D_METHOD("get_height_map_image"), &WaveSampler::get_height_map_image);
    
    ClassDB::bind_method(D_METHOD("set_detail_map", "texture"), &WaveSampler::set_detail_map);
    ClassDB::bind_method(D_METHOD("get_detail_map_image"), &WaveSampler::get_detail_map_image);

    // Detail viewport properties
    ClassDB::bind_method(D_METHOD("set_details_corner_position", "position"), &WaveSampler::set_details_corner_position);
    ClassDB::bind_method(D_METHOD("get_details_corner_position"), &WaveSampler::get_details_corner_position);
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "details_corner_position"), "set_details_corner_position", "get_details_corner_position");

    ClassDB::bind_method(D_METHOD("set_details_texture_size", "size"), &WaveSampler::set_details_texture_size);
    ClassDB::bind_method(D_METHOD("get_details_texture_size"), &WaveSampler::get_details_texture_size);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "details_texture_size", PROPERTY_HINT_RANGE, "1,1024,0.1"), "set_details_texture_size", "get_details_texture_size");

    ClassDB::bind_method(D_METHOD("set_details_vertex_spacing", "spacing"), &WaveSampler::set_details_vertex_spacing);
    ClassDB::bind_method(D_METHOD("get_details_vertex_spacing"), &WaveSampler::get_details_vertex_spacing);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "details_vertex_spacing", PROPERTY_HINT_RANGE, "0.01,10,0.01"), "set_details_vertex_spacing", "get_details_vertex_spacing");

    // Manual parameters (exposed for tweaking without material)
    ClassDB::bind_method(D_METHOD("set_mix_sharpness", "sharpness"), &WaveSampler::set_mix_sharpness);
    ClassDB::bind_method(D_METHOD("get_mix_sharpness"), &WaveSampler::get_mix_sharpness);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "mix_sharpness", PROPERTY_HINT_RANGE, "0,2,0.01"), "set_mix_sharpness", "get_mix_sharpness");

    ClassDB::bind_method(D_METHOD("set_height", "height"), &WaveSampler::set_height);
    ClassDB::bind_method(D_METHOD("get_height"), &WaveSampler::get_height);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "height", PROPERTY_HINT_RANGE, "0,100,0.1"), "set_height", "get_height");

    ClassDB::bind_method(D_METHOD("set_scale", "scale"), &WaveSampler::set_scale);
    ClassDB::bind_method(D_METHOD("get_scale"), &WaveSampler::get_scale);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "scale", PROPERTY_HINT_RANGE, "0.001,10,0.001"), "set_scale", "get_scale");

    ClassDB::bind_method(D_METHOD("set_speed", "speed"), &WaveSampler::set_speed);
    ClassDB::bind_method(D_METHOD("get_speed"), &WaveSampler::get_speed);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "speed", PROPERTY_HINT_RANGE, "0,10,0.1"), "set_speed", "get_speed");

    ClassDB::bind_method(D_METHOD("set_direction", "direction"), &WaveSampler::set_direction);
    ClassDB::bind_method(D_METHOD("get_direction"), &WaveSampler::get_direction);
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "direction"), "set_direction", "get_direction");

    ClassDB::bind_method(D_METHOD("set_normal_sample_range", "range"), &WaveSampler::set_normal_sample_range);
    ClassDB::bind_method(D_METHOD("get_normal_sample_range"), &WaveSampler::get_normal_sample_range);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "normal_sample_range", PROPERTY_HINT_RANGE, "0.01,10,0.01"), "set_normal_sample_range", "get_normal_sample_range");

    ClassDB::bind_method(D_METHOD("set_normal_softness", "softness"), &WaveSampler::set_normal_softness);
    ClassDB::bind_method(D_METHOD("get_normal_softness"), &WaveSampler::get_normal_softness);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "normal_softness", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_normal_softness", "get_normal_softness");

    ClassDB::bind_method(D_METHOD("set_max_height", "max_height"), &WaveSampler::set_max_height);
    ClassDB::bind_method(D_METHOD("get_max_height"), &WaveSampler::get_max_height);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_height", PROPERTY_HINT_RANGE, "0,100,0.1"), "set_max_height", "get_max_height");

    ClassDB::bind_method(D_METHOD("set_vertex_scaling", "scaling"), &WaveSampler::set_vertex_scaling);
    ClassDB::bind_method(D_METHOD("get_vertex_scaling"), &WaveSampler::get_vertex_scaling);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "vertex_scaling", PROPERTY_HINT_RANGE, "0.01,10,0.01"), "set_vertex_scaling", "get_vertex_scaling");

    // Methods
    ClassDB::bind_method(D_METHOD("refresh_from_material"), &WaveSampler::refresh_from_material);
    ClassDB::bind_method(D_METHOD("set_wave_time", "time"), &WaveSampler::set_wave_time);
    ClassDB::bind_method(D_METHOD("get_wave_height_xz", "world_xz", "time"), &WaveSampler::get_wave_height_xz);    
    ClassDB::bind_method(D_METHOD("get_wave_normal_xz", "world_xz", "time"), &WaveSampler::get_wave_normal_xz);
    ClassDB::bind_method(D_METHOD("get_wave_transform", "world_pos", "base_y", "ocean_global_transform"), &WaveSampler::get_wave_transform);
    ClassDB::bind_method(D_METHOD("is_valid"), &WaveSampler::is_valid);

    // Internal callback for async detail map fetch
    ClassDB::bind_method(D_METHOD("_fetch_detailmap_callback", "data"), &WaveSampler::_fetch_detailmap_callback);    
}

void WaveSampler::set_material(const Ref<ShaderMaterial> &p_material) {
    _material = p_material;
    _fetch_shader_parameters();
}

Ref<ShaderMaterial> WaveSampler::get_material() const {
    return _material;
}

void WaveSampler::set_height_map(const Ref<Texture2D> &p_texture) {
    if (p_texture.is_valid()) {
        _height_map = p_texture->get_image();
    } else {
        _height_map.unref();
    }

    emit_changed();
}

Ref<Image> WaveSampler::get_height_map_image() const {
    return _height_map;
}

void WaveSampler::set_detail_map(const Ref<Texture2D> &p_texture) {
    {
        std::lock_guard<std::mutex> lock(_detail_map_mutex);
        if (p_texture.is_valid()) {
            _detail_map = p_texture->get_image();
            _detail_map_texture = p_texture;
        } else {
            _detail_map.unref();
            _detail_map_texture.unref();
        }
    }
    
    emit_changed();
}

Ref<Image> WaveSampler::get_detail_map_image() const {
    return _detail_map;
}

void WaveSampler::set_details_corner_position(const Vector3 &p_position) {
    _details_corner_position = p_position;
    emit_changed();
}

Vector3 WaveSampler::get_details_corner_position() const {
    return _details_corner_position;
}

void WaveSampler::set_details_texture_size(float p_size) {
    _details_texture_size = p_size;
    emit_changed();
}

float WaveSampler::get_details_texture_size() const {
    return _details_texture_size;
}

void WaveSampler::set_details_vertex_spacing(float p_spacing) {
    _details_vertex_spacing = p_spacing;
    emit_changed();
}

float WaveSampler::get_details_vertex_spacing() const {
    return _details_vertex_spacing;
}

void WaveSampler::set_mix_sharpness(float p_sharpness) {
    _mix_sharpness = p_sharpness;
    emit_changed();
}

float WaveSampler::get_mix_sharpness() const {
    return _mix_sharpness;
}

void WaveSampler::set_height(float p_height) {
    _height = p_height;
    emit_changed();
}

float WaveSampler::get_height() const {
    return _height;
}

void WaveSampler::set_scale(float p_scale) {
    _scale = p_scale;
    emit_changed();
}

float WaveSampler::get_scale() const {
    return _scale;
}

void WaveSampler::set_speed(float p_speed) {
    _speed = p_speed;
    emit_changed();
}

float WaveSampler::get_speed() const {
    return _speed;
}

void WaveSampler::set_direction(const Vector2 &p_direction) {
    _direction = p_direction;
    emit_changed();
}

Vector2 WaveSampler::get_direction() const {
    return _direction;
}

void WaveSampler::set_normal_sample_range(float p_range) {
    _normal_sample_range = p_range;
    emit_changed();
}

float WaveSampler::get_normal_sample_range() const {
    return _normal_sample_range;
}

void WaveSampler::set_normal_softness(float p_softness) {
    _normal_softness = CLAMP(p_softness, 0.0f, 1.0f);
    emit_changed();
}

float WaveSampler::get_normal_softness() const {
    return _normal_softness;
}

void WaveSampler::set_max_height(float p_max) {
    _max_height = p_max;
    emit_changed();
}

float WaveSampler::get_max_height() const {
    return _max_height;
}

void WaveSampler::set_vertex_scaling(float p_scaling) {
    _vertex_scaling = p_scaling;
    emit_changed();
}

float WaveSampler::get_vertex_scaling() const {
    return _vertex_scaling;
}


#pragma region Material Refresh

void WaveSampler::_notification(int p_what) {
    switch (p_what) {
        case Node::NOTIFICATION_ENTER_TREE:
            _fetch_shader_parameters();
            break;
        case Node::NOTIFICATION_EXIT_TREE:
            // Clear cached data when exiting the tree to free memory
            _height_map.unref();
            _detail_map.unref();
            _height_map_texture.unref();
            _detail_map_texture.unref();
            break;

        case Node::NOTIFICATION_READY:
            // Initial fetch of shader parameters when the node is ready
            _fetch_shader_parameters();
            break;
        default:
            break;
    }
}


void WaveSampler::set_wave_time(float time) {
    _wave_time = time;
    if (_material.is_valid()) {
        _material->set_shader_parameter("WaveTime", time);
    }
}

void WaveSampler::refresh_from_material() {
    _fetch_shader_parameters();
}

void WaveSampler::_fetch_shader_parameters() {
    if (_material.is_null()) {
        return;
    }

    // Update the global shader parameter for detail map sampling callback
    RenderingServer *rs = RenderingServer::get_singleton();
    ERR_FAIL_NULL(rs);

    if (Engine::get_singleton()->is_editor_hint()) {
        // fetch synchronously in editor. async callback has synchronization problems and sometimes crashes out driver.
        if ( _detail_map.is_valid()==false ) {
            _detail_map = _detail_map_texture.is_valid() ? _detail_map_texture->get_image() : Ref<Image>();
        }
    } else {
        // Fetch async in game for speed
        _fetch_detailmap_async();

    }

    // Fetch shader uniform values
    Variant mix_sharpness = _material->get_shader_parameter("BaseWavesMixSharpness");
    if (mix_sharpness.get_type() == Variant::FLOAT) {
        _mix_sharpness = mix_sharpness;
    }

    Variant height = _material->get_shader_parameter("BaseWavesHeight");
    if (height.get_type() == Variant::FLOAT) {
        _height = height;
    }

    Variant scale = _material->get_shader_parameter("BaseWavesScale");
    if (scale.get_type() == Variant::FLOAT) {
        _scale = scale;
    }

    Variant speed = _material->get_shader_parameter("BaseWavesSpeed");
    if (speed.get_type() == Variant::FLOAT) {
        _speed = speed;
    }

    Variant direction = _material->get_shader_parameter("BaseWavesDirection");
    if (direction.get_type() == Variant::VECTOR2) {
        _direction = direction;
    }

    Variant normal_range = _material->get_shader_parameter("BaseWavesNormalSampleRange");
    if (normal_range.get_type() == Variant::FLOAT) {
        _normal_sample_range = normal_range;
    }

    Variant detail_normal_range = _material->get_shader_parameter("NormalSampleRange");
    if (detail_normal_range.get_type() == Variant::FLOAT) {
        _detail_normal_sample_range = detail_normal_range;
    }

    Variant normal_soft = _material->get_shader_parameter("BaseWavesNormalSoftness");
    if (normal_soft.get_type() == Variant::FLOAT) {
        _normal_softness = normal_soft;
    }

    Variant max_height = _material->get_shader_parameter("MaxWaveHeight");
    if (max_height.get_type() == Variant::FLOAT) {
        _max_height = max_height;
    }

    Variant vertex_scaling = _material->get_shader_parameter("VertexScaling");
    if (vertex_scaling.get_type() == Variant::FLOAT) {
        _vertex_scaling = vertex_scaling;
    }

    Variant height_map = _material->get_shader_parameter("BaseWavesHeightMap");
    if (height_map.get_type() == Variant::OBJECT) {
        Ref<Texture2D> tex = Object::cast_to<Texture2D>(static_cast<Object *>(height_map));
        _height_map_texture = tex;
    }

    // this only needs to be fetched once.
    if (_height_map_texture.is_valid() && _height_map.is_null()) {
        _fetch_heightmap();
    }
}

void WaveSampler::_fetch_heightmap() {
    if (_material.is_null()) {
        _height_map.unref();
        return;
    }

    Variant tex_variant = _material->get_shader_parameter("BaseWavesHeightMap");
    if (tex_variant.get_type() != Variant::OBJECT) {
        _height_map.unref();
        return;
    }

    Ref<Texture2D> tex = Object::cast_to<Texture2D>(static_cast<Object *>(tex_variant));
    if (tex.is_valid()) {
        // Synchronous fetch for now
        // TODO: When texture_get_data_async becomes available in godot-cpp, use it with _texture_data_callback
        _height_map = tex->get_image();
    } else {
        _height_map.unref();
    }
}

void WaveSampler::_fetch_detailmap_async() {
    // This is a bit hacky - we rely on the material to have a reference to the detail viewport texture, and we trigger an async read of it here. When the read completes, it will call back to _fetch_detailmap_callback where we canupdate our cached detail map image.
    if (_material.is_null()) {
        _detail_map.unref();
        return;
    }

    RenderingServer *rs = RenderingServer::get_singleton();
    ERR_FAIL_NULL(rs);

    RenderingDevice *rd = rs->get_rendering_device();
    ERR_FAIL_NULL(rd);

    if (_detail_map_texture.is_valid()) {
        RID texture_rid = _detail_map_texture->get_rid();
        if (!texture_rid.is_valid()) {
            _detail_map.unref();
            return;
        }

        // Get the RenderingDevice texture RID
        RID rd_texture_rid = rs->texture_get_rd_texture(texture_rid);
        if (!rd_texture_rid.is_valid()) {
            _detail_map.unref();
            return;
        }

        // Call texture_get_data_async through the Variant system since it's not exposed in godot-cpp
        // The method signature is: Error texture_get_data_async(RID p_texture, uint32_t p_layer, Callable p_callback)
        Callable callback = Callable(this, "_fetch_detailmap_callback");
        Variant result = rd->call("texture_get_data_async", rd_texture_rid, 0, callback);
        
        // Check if the call succeeded
        if (result.get_type() == Variant::INT) {
            Error err = (Error)(int)result;
            if (err != OK) {
                ERR_PRINT(vformat("Failed to start async texture fetch: %d", err));
                _detail_map.unref();
            }
        }
    } else {
        _detail_map.unref();
    }
}

void WaveSampler::_fetch_detailmap_callback(const PackedByteArray &p_data) {
    RenderingServer *rs = RenderingServer::get_singleton();
    ERR_FAIL_NULL(rs);

    if (_detail_map_texture.is_valid()) {
        RID texture_rid = _detail_map_texture->get_rid();
        Image::Format format = (Image::Format)RenderingServer::get_singleton()->texture_get_format(texture_rid);
        
        // Create image from async data
        Ref<Image> image;
        image.instantiate();
        image->set_data(_detail_map_texture->get_width(), _detail_map_texture->get_height(), false, format, p_data);
        
        // Thread-safe assignment
        std::lock_guard<std::mutex> lock(_detail_map_mutex);
        _detail_map = image;
    }
}

#pragma endregion

#pragma region Texture Sampling

Vector2 WaveSampler::_vec2_xz(const Vector3 &v) const {
    return Vector2(v.x, v.z);
}

Vector2 WaveSampler::_clamp_uv(const Vector2 &uv, float min_val, float max_val) const {
    return Vector2(
        CLAMP(uv.x, min_val, max_val),
        CLAMP(uv.y, min_val, max_val)
    );
}

Color WaveSampler::_sample_image(const Ref<Image> &p_image, const Vector2 &uv, const Color &p_default_color) const {
    if (p_image.is_null()) {
        return p_default_color;
    }

    int width = p_image->get_width();
    int height = p_image->get_height();
    if (width == 0 || height == 0) {
        return p_default_color;
    }

    // Handle wrapping (fract in GLSL)
    float u = Math::fmod(uv.x, 1.0f);
    float v = Math::fmod(uv.y, 1.0f);
    if (u < 0.0f) u += 1.0f;
    if (v < 0.0f) v += 1.0f;

    int x = static_cast<int>(u * width) % width;
    int y = static_cast<int>(v * height) % height;

    return p_image->get_pixel(x, y);
}

Color WaveSampler::_sample_texture(const Vector2 &uv) const {
    return _sample_image(_height_map, uv, Color(0.5f, 0.5f, 0.5f, 1.0f));
}

Color WaveSampler::_sample_detail_texture(const Vector2 &uv) const {
    std::lock_guard<std::mutex> lock(_detail_map_mutex);
    return _sample_detail_texture_unlocked(uv);
}

Color WaveSampler::_sample_detail_texture_unlocked(const Vector2 &uv) const {
    // NOTE: This is called with _detail_map_mutex already locked!
    return _sample_image(_detail_map, uv, Color(0.0f, 0.0f, 0.0f, 1.0f));
}

Color WaveSampler::_sample_detail_effects_layer(const Vector3 &world_position) const {
    std::lock_guard<std::mutex> lock(_detail_map_mutex);
    
    if (_detail_map.is_null()) {
        return Color(0.0f, 0.0f, 0.0f, 1.0f);
    }

    // Replicate shader code
    Vector3 delta = world_position - _details_corner_position;
    Vector2 uv = _vec2_xz(delta) / _details_texture_size;

    // Check if UV is in valid range
    if (uv != _clamp_uv(uv, 0.0f, 1.0f)) {
        return Color(0.0f, 0.0f, 0.0f, 0.0f);
    }

    uv = _clamp_uv(uv, 0.0f, 1.0f);
    
    // For now, skip FadeEffectAtEdges logic - can be added later if needed
    // Call unlocked version since we already hold the mutex
    return _sample_detail_texture_unlocked(uv);
}

Vector4 WaveSampler::_retrieve_normal_and_height(const Vector3 &center_world_position, int channel) const {
    Color detail_effects_layer_data = _sample_detail_effects_layer(center_world_position);
    
    // Sample at center + up (using detail normal sample range)
    Vector3 position_up = Vector3(0.0f, 0.0f, _detail_normal_sample_range);
    float temp_height = _sample_detail_effects_layer(center_world_position + position_up)[channel] * detail_effects_layer_data.a;
    position_up.y += temp_height;

    // Sample at center + right (using detail normal sample range)
    Vector3 position_right = Vector3(_detail_normal_sample_range, 0.0f, 0.0f);
    temp_height = _sample_detail_effects_layer(center_world_position + position_right)[channel] * detail_effects_layer_data.a;
    position_right.y += temp_height;

    // Center height
    float center_y = detail_effects_layer_data[channel] * detail_effects_layer_data.a;
    Vector3 center = Vector3(0.0f, center_y, 0.0f);

    // Calculate normal
    Vector3 updir = position_up - center;
    Vector3 rightdir = position_right - center;
    Vector3 normal = updir.cross(rightdir).normalized();
    
    return Vector4(normal.x, normal.y, normal.z, center.y);
}

float WaveSampler::_sample_wave_height(const Vector2 &world_xz, float time) const {
    // Emulate shader: BaseCenter = (world_position) * vec3(BaseWavesScale, 1.0, BaseWavesScale)
    // In shader, this is (NODE_POSITION_WORLD + VERTEX), but for sampling we just use world_xz
    Vector2 base_center_xz = world_xz * _scale;
    
    // BaseWavesUVAdjustment = BaseWavesDirection.normalized() * TIME * BaseWavesSpeed
    Vector2 uv_adjustment = _direction.normalized() * time * _speed;
    
    // Sample the heightmap at the adjusted UV
    Vector2 sample_uv = base_center_xz + uv_adjustment;
    Color sample = _sample_texture(sample_uv);
    
    // In shader: .r * BaseWavesHeight
    float wave_height = sample.r * _height;
    
    return wave_height;
}

bool WaveSampler::is_valid() const {
    return _height_map.is_valid();
}

float WaveSampler::get_wave_height_xz(const Vector2 &world_xz, float time) const {
    return _sample_wave_height(world_xz, time);
}

float WaveSampler::get_wave_height(const Vector3 &world_pos, float time) const {
    return _sample_wave_height(Vector2(world_pos.x, world_pos.z), time);
}

Vector3 WaveSampler::get_wave_normal_xz(const Vector2 &world_xz, float time) const {
    // Emulate shader normal calculation:
    // BaseCenter = (world_position) * vec3(BaseWavesScale, 1.0, BaseWavesScale)
    Vector2 base_center_xz = world_xz * _scale;
    Vector2 uv_adjustment = _direction.normalized() * time * _speed;
    
    // Sample three points: center, up (+Z), and right (+X)
    Vector2 center_uv = base_center_xz + uv_adjustment;
    Vector2 up_uv = (base_center_xz + Vector2(0.0f, _normal_sample_range)) + uv_adjustment;
    Vector2 right_uv = (base_center_xz + Vector2(_normal_sample_range, 0.0f)) + uv_adjustment;
    
    float center_height = _sample_texture(center_uv).r * _height;
    float up_height = _sample_texture(up_uv).r * _height;
    float right_height = _sample_texture(right_uv).r * _height;
    
    // Build 3D positions
    Vector3 base_center(0.0f, center_height, 0.0f);
    Vector3 base_waves_up(0.0f, up_height, _normal_sample_range);
    Vector3 base_waves_right(_normal_sample_range, right_height, 0.0f);
    
    // Calculate direction vectors
    Vector3 base_up_dir = base_waves_up - base_center;
    Vector3 base_right_dir = base_waves_right - base_center;
    
    // Cross product: updir.cross(rightdir)
    Vector3 normal = base_up_dir.cross(base_right_dir).normalized();
    
    // Apply softness: lerp toward (0, 1, 0)
    normal = normal.lerp(Vector3(0.0f, 1.0f, 0.0f), _normal_softness).normalized();
    
    return normal;
}

Vector3 WaveSampler::get_wave_normal(const Vector3 &world_pos, float time) const {
    return get_wave_normal_xz(Vector2(world_pos.x, world_pos.z), time);
}

Transform3D WaveSampler::get_wave_transform(const Vector3 &world_pos, float base_y, const Transform3D &ocean_global_transform) const {
    Transform3D error_xform = Transform3D(Basis(), Vector3(0,-INFINITY,0));
    ERR_FAIL_COND_V_MSG(_height_map.is_null(), error_xform, "Height map is null");

    // print this once to avoid spamming console.
    bool detail_map_is_null = false;
    {
        std::lock_guard<std::mutex> lock(_detail_map_mutex);
        detail_map_is_null = _detail_map.is_null();
    }
    
    if (detail_map_is_null) {
        if (_detail_missing_reported == false) {
            WARN_PRINT("Detail map is null - Either waiting for first fetch or it has not been assigned.");
            const_cast<WaveSampler*>(this)->_detail_missing_reported = true;
        }
        return error_xform;
    }

    // Match shader implementation
    Vector3 pos = world_pos;
    pos.y = base_y;  // Fix input height at ocean height    

    Transform3D xform;
    xform.origin = pos;
    xform.basis = Basis();  // Identity
    
    // Recreate the wave height calc from the shader
    Vector3 NODE_POSITION_WORLD = Vector3(0, base_y, 0);  // Ocean's global position
    Vector3 VERTEX = pos;  // In local space, this would be global_transform.inverse() * pos
    float TIME = _wave_time;
    Vector3 NORMAL = Vector3(0, 1, 0);
    
    // Sample detail effects layer
    Color detail_effects_layer_data = _sample_detail_effects_layer(NODE_POSITION_WORLD + VERTEX);
    detail_effects_layer_data.b *= detail_effects_layer_data.a;
    
    // Calculate dynamic height blend
    float dynamic_height_blend = Math::smoothstep(0.0f, _mix_sharpness, detail_effects_layer_data.b);
    
    // Base waves calculation
    Vector3 base_center = (NODE_POSITION_WORLD + VERTEX) * Vector3(_scale, 1.0f, _scale);
    Vector2 base_waves_uv_adjustment = _direction.normalized() * TIME * _speed;
    
    // Sample three points for normal calculation
    Vector3 base_waves_up = base_center + Vector3(0.0f, 0.0f, _normal_sample_range);
    base_waves_up.y += _sample_texture(Vector2(base_waves_up.x, base_waves_up.z) + base_waves_uv_adjustment).r * _height;
    
    Vector3 base_waves_right = base_center + Vector3(_normal_sample_range, 0.0f, 0.0f);
    base_waves_right.y += _sample_texture(Vector2(base_waves_right.x, base_waves_right.z) + base_waves_uv_adjustment).r * _height;
    
    float center_height = _sample_texture(Vector2(base_center.x, base_center.z) + base_waves_uv_adjustment).r * _height;
    base_center.y += center_height;
    
    // Calculate base wave normal
    Vector3 base_updir = base_waves_up - base_center;
    Vector3 base_rightdir = base_waves_right - base_center;
    NORMAL = base_updir.cross(base_rightdir).normalized().lerp(NORMAL, _normal_softness);
    
    // Apply base wave height with dynamic blend
    VERTEX.y += Math::lerp(center_height, 0.0f, dynamic_height_blend * 0.2f);
    
    // Add detail effects
    float current_height = VERTEX.y;
    current_height += detail_effects_layer_data.b * CLAMP(detail_effects_layer_data.b, 0.0f, 1.0f) * _max_height;
    
    if (current_height != VERTEX.y) {
        Vector4 normal_and_height = _retrieve_normal_and_height(NODE_POSITION_WORLD + VERTEX, 2);
        NORMAL = NORMAL.lerp(Vector3(normal_and_height.x, normal_and_height.y, normal_and_height.z), dynamic_height_blend);
        VERTEX.y = MAX(VERTEX.y, VERTEX.y + normal_and_height.w);
    }
    
    // Apply VertexScaling division (shader: VERTEX.y /= VertexScaling;)
    // VERTEX.y /= _vertex_scaling;
    
    // Set final transform
    xform.origin = VERTEX;
    
    // Align transform with Y-axis using the normal
    Vector3 up = NORMAL;
    Vector3 forward = Vector3(0.0f, 0.0f, 1.0f);
    if (Math::abs(up.dot(forward)) > 0.999f) {
        forward = Vector3(1.0f, 0.0f, 0.0f);
    }
    Vector3 right = up.cross(forward).normalized();
    forward = right.cross(up).normalized();
    xform.basis = Basis(right, up, forward);
    
    return xform;
}

#pragma endregion