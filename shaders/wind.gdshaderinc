/*
Wind Include

This include is use to harmonize wind behaviors across various shaders in Halyard.
If you want to integrate your own wind behaviors use this as your starting point.
*/

// For testing you can disable the globals and use the local parameters to test.
#define GLOBAL_WIND

#ifdef GLOBAL_WIND
// Wind angle in radians. 0 is global Z+ direction, increasing clockwise when looking from above.
global uniform float WindAngle : hint_range(0.0, 6.283185307179586, 0.1) = 0.0;
// Wind speed, in meters/sec. This is the base speed that gusts are added on top of.
global uniform float WindSpeed : hint_range(0.0, 10.0, 0.1) = 1.0;
// Wind gust strength, in meters/sec. This is the maximum additional speed that gusts can add on top of the base WindSpeed.
global uniform float WindGust : hint_range(0.0, 10.0, 0.1) = 1.0;
// WindMap is a texture that provides gust information. Scrolls in the WindAngle direction.
global uniform sampler2D WindMap : hint_default_white;
// WindScroll is the UV offset for the wind over time. Updated in code.
global uniform vec2 WindScroll = vec2(0., 0.);
#else
group_uniforms Wind;
uniform float WindAngle : hint_range(0.0, 6.283185307179586, 0.1) = 0.0;
uniform float WindSpeed : hint_range(0.0, 10.0, 0.1) = 1.0;
uniform float WindGust : hint_range(0.0, 10.0, 0.1) = 1.0;
uniform sampler2D WindMap : hint_default_white, filter_linear, repeat_enable;
#endif

// Wind direction as a 2D vector. 0 degrees is global Z+ direction.
vec2 wind_direction() {
    return vec2(sin(WindAngle), cos(WindAngle));
}

// Wind strength, acts as a multiplier for gusts over the base WindSpeed
float wind_strength(vec3 world_pos) {
    float gust = WindGust * texture(WindMap, WindScroll + world_pos.xz/128.0).r;
    return WindSpeed + gust;
}

// Wind speed and direction as a vector.
vec3 wind_vector(vec3 world_pos) {
    vec2 wind_dir = wind_direction();
    vec2 wind = wind_dir * wind_strength(world_pos);
    return vec3(wind.x, 0.0, wind.y);
}