/* WaveSampler

A Resource class that samples wave height and normal from a shader material's parameters.
This allows wave calculations to be shared between different systems (Ocean, other water bodies, etc.)

From Bonkahe's Detail Environment Interactions plugin, used with permission.
https://github.com/Bonkahe/DetailEnviromentInteractions/

*/
#pragma once

#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/classes/shader_material.hpp>
#include <godot_cpp/classes/texture2d.hpp>
#include <godot_cpp/core/binder_common.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <mutex>

using namespace godot;

class WaveSampler : public Resource {
	GDCLASS(WaveSampler, Resource)

private:
	// Source material to fetch parameters from
	Ref<ShaderMaterial> _material;

	// Cached shader parameters
	Ref<Image> _height_map; // read once at load or when material is set
	Ref<Image> _detail_map; // read every frame in async callback
	float _mix_sharpness = 0.5f;
	float _height = 1.0f;
	float _scale = 0.1f;
	float _speed = 1.0f;
	Vector2 _direction = Vector2(1.0f, 0.0f);
	float _normal_sample_range = 0.1f; // BaseWavesNormalSampleRange - for base waves
	float _detail_normal_sample_range = 0.25f; // NormalSampleRange - for detail effects
	float _normal_softness = 0.5f;
	float _max_height = 1.0f;
	float _vertex_scaling = 1.0f; // VertexScaling - matches shader uniform
	float _wave_time = 0.0f; // Current wave time

	// references to the textures used for sampling
	Ref<Texture2D> _height_map_texture;

	// global shader params
	Ref<Texture2D> _detail_map_texture;
	Vector3 _details_corner_position = Vector3(0, 0, 0);
	float _details_texture_size = 128.0f;
	float _details_vertex_spacing = 1.0f;

	// To avoid spamming errors if material is missing parameters
	bool _detail_missing_reported = false;

	// Thread safety for async detail map updates
	mutable std::mutex _detail_map_mutex;

	// Internal methods
	void _fetch_shader_parameters();
	void _fetch_heightmap();
	void _fetch_detailmap_async();
	void _fetch_detailmap_callback(const PackedByteArray &p_data);
	Color _sample_texture(const Vector2 &uv) const;
	Color _sample_detail_texture(const Vector2 &uv) const;
	Color _sample_detail_texture_unlocked(const Vector2 &uv) const; // Called with mutex already locked

	// Common image sampling helper
	Color _sample_image(const Ref<Image> &p_image, const Vector2 &uv, const Color &p_default_color) const;

	// Detail effects layer sampling
	Color _sample_detail_effects_layer(const Vector3 &world_position) const;
	Vector4 _retrieve_normal_and_height(const Vector3 &center_world_position, int channel) const;

	// Helper methods
	Vector2 _vec2_xz(const Vector3 &v) const;
	Vector2 _clamp_uv(const Vector2 &uv, float min_val, float max_val) const;

	// Core wave calculation - samples height at a single point
	float _sample_wave_height(const Vector2 &world_xz, float time) const;

protected:
	static void _bind_methods();

public:
	WaveSampler();
	~WaveSampler() override;

	void _notification(int p_what);

	// Material property
	void set_material(const Ref<ShaderMaterial> &p_material);
	Ref<ShaderMaterial> get_material() const;

	// Manual parameter setters (for use without a material)
	void set_height_map(const Ref<Texture2D> &p_texture);
	Ref<Image> get_height_map_image() const;

	void set_detail_map(const Ref<Texture2D> &p_texture);
	Ref<Image> get_detail_map_image() const;

	void set_details_corner_position(const Vector3 &p_position);
	Vector3 get_details_corner_position() const;

	void set_details_texture_size(float p_size);
	float get_details_texture_size() const;

	void set_details_vertex_spacing(float p_spacing);
	float get_details_vertex_spacing() const;

	void set_mix_sharpness(float p_sharpness);
	float get_mix_sharpness() const;

	void set_height(float p_height);
	float get_height() const;

	void set_scale(float p_scale);
	float get_scale() const;

	void set_speed(float p_speed);
	float get_speed() const;

	void set_direction(const Vector2 &p_direction);
	Vector2 get_direction() const;

	void set_normal_sample_range(float p_range);
	float get_normal_sample_range() const;

	void set_normal_softness(float p_softness);
	float get_normal_softness() const;

	void set_max_height(float p_max);
	float get_max_height() const;

	void set_vertex_scaling(float p_scaling);
	float get_vertex_scaling() const;

	// Refresh cached parameters from material
	void refresh_from_material();

	// Set wave time parameter in material
	void set_wave_time(float time);

	// Main sampling methods
	// Returns wave height displacement at the given world XZ position
	float get_wave_height_xz(const Vector2 &world_xz, float time) const;
	float get_wave_height(const Vector3 &world_pos, float time) const;

	// Returns surface normal at the given world XZ position
	Vector3 get_wave_normal_xz(const Vector2 &world_xz, float time) const;
	Vector3 get_wave_normal(const Vector3 &world_pos, float time) const;

	// Returns a full transform (position + orientation) for the wave surface
	Transform3D get_wave_transform(const Vector3 &world_pos, float base_y, const Transform3D &ocean_global_transform) const;

	// Check if the sampler has valid data
	bool is_valid() const;
};
