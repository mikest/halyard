/* Ocean

A class that provides ocean wave simulation with height displacement based on a noise texture.
Derives from LiquidArea and overrides update_transforms_for_points to provide wave-based height sampling.

The wave simulation is delegated to a WaveSampler resource which recreates the shader logic
used to calculate wave height displacement, allowing for accurate physics sampling of the wave surface.
*/
#pragma once

#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/shader_material.hpp>
#include <godot_cpp/core/binder_common.hpp>

#include "clip_map.h"
#include "liquid_area.h"
#include "ocean_detailer.h"
#include "wave_sampler.h"

using namespace godot;

class OceanArea : public LiquidArea {
	GDCLASS(OceanArea, LiquidArea)

public:
	enum OceanUpdate {
		DISABLED = 0, // Don't follow
		CAMERA = 1, // Follow camera position
		DETAILER_TARGET = 2, // Follow ocean detailer target
	};

private:
	// Wave sampling delegated to resource
	Ref<WaveSampler> _wave_sampler;

	// Visual clipmap (optional - can be null if using GDScript visuals)
	ClipMapInstance *_clipmap_instance = nullptr;
	OceanDetailer *_ocean_detailer = nullptr;
	OceanUpdate _ocean_update = OceanUpdate::CAMERA;

	// Rendering properties
	float _vertex_scaling = 1.0f;

	// Tide properties
	float _tidal_override = -1.0f; // if < 0, tide follows external source
	float _low_tide = -5.0f;
	float _high_tide = 5.0f;
	float _tide_phase = 0.5f;

	// Internal methods
	Transform3D _get_wave_transform_at_position(const Vector3 &pos, float time) const;
	float _get_current_time() const;
	void _update_clipmap_position();
	void _frame_post_draw();

protected:
	static void _bind_methods();
	void _notification(int p_what);

	// Override from LiquidArea - provides wave-based transforms
	void _internal_update_transforms_for_points(const PackedVector3Array &global_points, TypedArray<Transform3D> r_transforms) const override;

public:
	OceanArea();
	~OceanArea() override;

	// Static accessor to find Ocean in scene tree
	static OceanArea *get_ocean_area();

	// Wave sampler property
	void set_wave_sampler(const Ref<WaveSampler> &p_sampler);
	Ref<WaveSampler> get_wave_sampler() const;

	// Clipmap properties
	void set_clipmap_instance(ClipMapInstance *p_instance);
	ClipMapInstance *get_clipmap_instance() const;

	// Detailer properties
	void set_ocean_detailer(OceanDetailer *p_instance);
	OceanDetailer *get_ocean_detailer() const;

	void set_ocean_update(OceanUpdate p_mode);
	OceanUpdate get_ocean_update() const;

	void set_vertex_scaling(float p_scale);
	float get_vertex_scaling() const;

	// Tide properties
	void set_tidal_override(float p_override);
	float get_tidal_override() const;

	void set_low_tide(float p_low);
	float get_low_tide() const;

	void set_high_tide(float p_high);
	float get_high_tide() const;

	void set_tide_phase(float p_phase);
	float get_tide_phase() const;

	// Convenience: get current tide height
	float get_tide_height() const;
};

VARIANT_ENUM_CAST(OceanArea::OceanUpdate);
