/* OceanDetailer

A SubViewport-based class that provides high-resolution ocean detail rendering.
Creates a Camera3D child at runtime to capture ocean surface details that can be
used for shader effects, foam generation, or other rendering techniques.

Copyright (c) 2026 M. Estee. - MIT License
*/
#pragma once

#include <godot_cpp/classes/sub_viewport.hpp>
#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/environment.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/core/binder_common.hpp>

using namespace godot;

class OceanDetailer : public SubViewport {
    GDCLASS(OceanDetailer, SubViewport)

public:
    // Constants
    static const int WAVE_CULL_MASK = 1 << (19-1);

private:
    // Runtime-created camera and environment
    Camera3D *_camera = nullptr;
    
    // Properties
    float _render_size = 1024.0f;
    float _reposition_rate = 0.1f;  // How often to reposition (in seconds)
    float _snap_step = 1.0f;         // Grid snapping distance
    Node3D *_follow_target = nullptr;
    
    // Tracking
    Vector3 _last_follow_position = Vector3(0, 0, 0);

    // Internal methods
    void _global_shader_update();

protected:
    static void _bind_methods();
    void _notification(int p_what);

public:
    OceanDetailer();
    ~OceanDetailer() override;

    // Property setters/getters
    void set_render_size(float p_size);
    float get_render_size() const;
    
    void set_reposition_rate(float p_rate);
    float get_reposition_rate() const;
    
    void set_snap_step(float p_step);
    float get_snap_step() const;
    
    void set_follow_target(Node3D *p_target);
    Node3D *get_follow_target() const;
    
    // Position update
    void update_position(const Vector3 &p_global_position);
    
    // Readonly getters for shader parameters (for inspector visibility)
    float get_detail_viewport_texture_size() const;
    Vector3 get_detail_viewport_texture_corner_position() const;
    float get_detail_viewport_vertex_spacing() const;
    
    // Camera accessor
    Camera3D *get_camera() const;

    // called on extension startup
    static void _global_shader_init();
};
