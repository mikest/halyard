// from https://godotshaders.com/shader/stylized-toon-water/

shader_type spatial;
render_mode cull_disabled, blend_add, unshaded;
uniform float time_speed = 1.0;
//time specifically for the wave noise texture
uniform float surface_speed = 1.0;
uniform float spin = 0.0; //Twisting motion of the water
uniform float brightness = 0.6;
uniform float color_intensity = 0.0;
//Tiling frequency of the noise accross the mesh
uniform float horizontal_frequency = 2.0;
uniform float vertical_frequency = 2.0;
//overall size muliplier
uniform float size = 3.0;
//affects total size
uniform float banding_bias = 0.6;

uniform sampler2D wave_texture;
uniform sampler2D noise_texture;
//wave height, use for ocean waves
uniform float wave_height = 0.5;
//water surface height variation based on the noise texture
uniform float texture_height = 0.5;
//preset band colors
uniform vec4 color1 : source_color = vec4(0.59, 0.761, 1.0, 1.0);
uniform vec4 color2 : source_color = vec4(0.274, 0.474, 0.98, 1.0);
uniform vec4 color3 : source_color = vec4(0.059, 0.389, 0.85, 1.0);
uniform vec4 color4 : source_color = vec4(0.0, 0.267, 1.0, 1.0);

void vertex() {
	float time = -TIME * time_speed;
	VERTEX += NORMAL * wave_height * texture(wave_texture, vec2(UV.x + time * surface_speed, UV.y + time * surface_speed)).r;
	VERTEX += NORMAL * texture_height * texture(noise_texture,vec2(UV.x * horizontal_frequency + spin * (time /2.0), (UV.y * vertical_frequency) + time)).r;;
}

void fragment() {
	float time = -TIME * time_speed;

// Calculate dot product of normals and combine with noise texture value
	float normal_facing = dot(NORMAL, VIEW);
	float noise_value = texture(noise_texture,vec2(UV.x * horizontal_frequency + spin * (time /2.0), (UV.y * vertical_frequency) + time)).r;
	normal_facing += (noise_value -0.5 + size) * 0.3;

	float band = normal_facing * 3.0 * banding_bias;
	vec4 band_color = vec4(0,0,0,0);
	if (band <= 1.5) {
		discard;
	}
	else if(band <= 2.0){
		band_color = mix(color1, color2, -0.01 / (band-2.01)); //Mixes the color bands to make a slight gradient
	}
	else if (band <= 2.5) {
		band_color = mix(color2, color3, -0.01 / (band-2.51));
	}
	else if (band <= 2.9) {
		band_color = mix(color3, color4, -0.01 / (band-2.91));
	}
	else if (band >= 0.0) {
		band_color = color4;
	}
	//Final color adjestment
	ALBEDO = clamp(brightness * (vec3(1.0, 1.0, 1.0) - (band_color.xyz * -color_intensity)) * band_color.xyz, vec3(0.0, 0.0, 0.0), vec3(brightness, brightness, brightness));
}
