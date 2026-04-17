/*
Plant Wind Shader

This uses the red & green vertex color channels to drive separate wind effects for branches and leaves, with parameters for amplitude, wavelength, and speed of the underlying LFO. The effects can be mixed together or applied separately by adjusting the wind behavior for the specific plant.

Is integrated with the global Wind properties.
*/

shader_type spatial;
render_mode cull_disabled;

// Base material properties
uniform sampler2D texture_albedo : hint_default_white, source_color, filter_linear_mipmap, repeat_enable;
uniform bool enable_backlight = true;
uniform bool enable_sss = false;
uniform sampler2D texture_backlight : hint_default_white, source_color, filter_linear_mipmap, repeat_enable;

uniform float specular : hint_range(0.0, 1.0, 0.01) = 0.33;
uniform float roughness : hint_range(0.0, 1.0) = 1.0;
uniform float metallic : hint_range(0.0, 1.0, 0.01) = 0.0;

group_uniforms RedBranch;
uniform float red_branch_gain: hint_range(-1.0, 1.0, 0.01) = 0.0;
uniform float red_branch_lfo : hint_range(0.0, 10.0, 0.001) = 1.0;
uniform float red_branch_wavelength : hint_range(0.0, 10.0, 0.001) = 1.0;
uniform float red_branch_amplitude : hint_range(0.0, 10.0, 0.001) = 1.0;

group_uniforms GreenLeaf;
uniform float green_leaf_gain: hint_range(-1.0, 1.0, 0.01) = 0.0;
uniform float green_leaf_lfo : hint_range(0.0, 10.0, 0.001) = 1.0;
uniform float green_leaf_wavelength : hint_range(0.0, 10.0, 0.001) = 1.0;
uniform float green_leaf_amplitude : hint_range(0.0, 10.0, 0.001) = 1.0;

group_uniforms Mask;
uniform float sphere_mask_radius : hint_range(0.0, 10.0, 0.001) = 0.0;
uniform float green_cylinder_mask_radius : hint_range(0.0, 10.0, 0.001) = 0.0;
uniform float red_height_mask_height : hint_range(0.0, 10.0, 0.001) = 0.0;

// Distance fade settings
group_uniforms Fade;
uniform bool enable_fade = false; // Toggle for enabling/disabling fade effect
uniform float fade_start_distance : hint_range(0.0, 200.0) = 50.0; // Distance at which fading starts
uniform float fade_length : hint_range(1.0, 50.0) = 20.0; // Distance over which fading occurs

#include "wind.gdshaderinc"
uniform bool enable_wind = true;

group_uniforms Debug;
uniform bool debug_color = false;
uniform bool debug_mask = false;
uniform bool debug_speed = false;

#define LINEAR_TRANSLATE 0
#define RADIAL_TWIST 1
#define VERTICAL_BEND 2
#define CURLING 3

#define GREEN_LEAF_EFFECT VERTICAL_BEND
#define RED_BRANCH_EFFECT CURLING

void vertex() {
    if (enable_wind) {
		vec2 dir_2d = wind_direction();
		vec3 dir = vec3(dir_2d.x, 0.0, dir_2d.y);
		dir = (inverse(MODEL_MATRIX) * vec4(dir, 0.0)).xyz;
		vec3 pos = NODE_POSITION_WORLD + VERTEX;
		float strength = wind_strength(pos);
		float sway = strength / 1.0;

		
#if RED_BRANCH_EFFECT==LINEAR_TRANSLATE
		// Red channel LFO: XZ-plane sway along wind direction, propagating up the Y axis
		// Traveling wave with zero crossing at VERTEX.y == 0 (node origin)
		float red_phase = (VERTEX.y * PI / red_branch_wavelength) + (TIME * red_branch_lfo);
		vec3 red_lfo = dir * sin(red_phase) * COLOR.r * red_branch_amplitude * strength;
#elif RED_BRANCH_EFFECT==CURLING
		float red_phase = (PI / red_branch_wavelength) + (TIME * red_branch_lfo) + strength;
		
		// Curling: progressive rotation where angle grows with height from base
        vec3 curl_axis = normalize(cross(vec3(0.0, 1.0, 0.0), dir));
        float d = max(VERTEX.y, 0.0);
        float curl_angle = sin(red_phase) * (COLOR.r + red_branch_gain) * red_branch_amplitude * sway * d;
        float rc = cos(curl_angle);
        float rs = sin(curl_angle);
        vec3 red_lfo = VERTEX * rc + cross(curl_axis, VERTEX) * rs + curl_axis * dot(curl_axis, VERTEX) * (1.0 - rc) - VERTEX;
#endif

		// Green channel LFO
		float radial_dist = length(VERTEX);
		float green_phase = (radial_dist * PI / green_leaf_wavelength) + (TIME * green_leaf_lfo * strength);

#if GREEN_LEAF_EFFECT!=LINEAR_TRANSLATE
		float green_angle = sin(green_phase) * (COLOR.g + green_leaf_gain) * green_leaf_amplitude * sway * radians(10.0);
		float gc = cos(green_angle);
		float gs = sin(green_angle);
#endif
#if GREEN_LEAF_EFFECT==RADIAL_TWIST
		// Radial twist: rotate vertex around Y axis
		vec3 green_lfo = vec3(VERTEX.x * gc - VERTEX.z * gs, VERTEX.y, VERTEX.x * gs + VERTEX.z * gc) - VERTEX;

#elif GREEN_LEAF_EFFECT==VERTICAL_BEND
		// Rotate vertex up/down around axis perpendicular to radial direction
		vec3 pivot = normalize(cross(VERTEX, vec3(0.0, 1.0, 0.0)));
		// Rodrigues' rotation: rotate VERTEX around pivot by green_angle
		vec3 green_lfo = VERTEX * gc + cross(pivot, VERTEX) * gs + pivot * dot(pivot, VERTEX) * (1.0 - gc) - VERTEX;

#elif GREEN_LEAF_EFFECT==LINEAR_TRANSLATE
		vec3 green_lfo = normalize(VERTEX) *  sin(green_phase) * COLOR.g * green_leaf_amplitude * strength;
#endif

		// Spherical distance from node origin for masking
		float sphere_dist = length(VERTEX);
		float cylinder_dist = length(vec3(VERTEX.x, 0, VERTEX.z));

        float sphere_mask = smoothstep(0.0, sphere_mask_radius, sphere_dist);
        float green_cylinder_mask = smoothstep(0.0, green_cylinder_mask_radius, cylinder_dist);
        float red_height_mask = smoothstep(0.0, red_height_mask_height, VERTEX.y);

		vec3 lfo = (red_lfo * red_height_mask + green_lfo * green_cylinder_mask) * sphere_mask;
		VERTEX += lfo;

        COLOR = debug_mask ? vec4(red_height_mask, green_cylinder_mask, sphere_mask,1.0) : COLOR; // Visualize wind effect if debug_color is enabled
		COLOR = debug_speed ? vec4(strength, strength, strength, 1.0) : COLOR;
    }
}

void fragment() {
    vec2 base_uv = UV;
//
    // Sample textures
    vec4 albedo_tex = texture(texture_albedo, base_uv);

    // Compute base material properties
    ALBEDO = (debug_color || debug_mask || debug_speed) ? COLOR.rgb : albedo_tex.rgb;
    METALLIC = metallic;
    SPECULAR = specular;
    ROUGHNESS = roughness;
	
	// backlighting
	vec4 backlight_tex = texture(texture_backlight, base_uv);
	BACKLIGHT = enable_backlight ? backlight_tex.rgb : vec3(0,0,0);
	
	// sub-surface scattering
	SSS_TRANSMITTANCE_COLOR = enable_sss ? backlight_tex : vec4(1,1,1,0);
	SSS_TRANSMITTANCE_DEPTH = enable_sss ? 1.0 : 1.0;
	SSS_TRANSMITTANCE_BOOST = enable_sss ? 0.5 : 0.0;
	SSS_STRENGTH = enable_sss ? 1.0 : 0.0;

    //// Compute fade effect only if enabled
    float fade = 1.0; // Default fully visible
    if (enable_fade) {
        float dist = length(VERTEX);
        fade = clamp(1.0 - (dist - fade_start_distance) / fade_length, 0.0, 1.0);
    }
	
    // Apply fading to alpha
    ALPHA = albedo_tex.a * fade;
	ALPHA_SCISSOR_THRESHOLD = 0.5;
}
