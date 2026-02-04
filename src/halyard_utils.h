#pragma once

#include <godot_cpp/variant/packed_vector3_array.hpp>

using namespace godot;

namespace halyard {

/// Calculate the full submerged depth based on probe extents in local space
/// This represents the depth at which all probes would be fully submerged
/// Returns a negative value (depth below surface)
inline float calculate_full_submerged_depth(const godot::PackedVector3Array &probes) {
    if (probes.size() == 0) {
        return 0.0f;
    }
    
    float min_probe_depth = 0.0f;
    float max_probe_depth = 0.0f;
    
    for (int i = 0; i < probes.size(); i++) {
        const godot::Vector3 &probe = probes[i];
        if (probe.y < min_probe_depth) {
            min_probe_depth = probe.y;
        }
        if (probe.y > max_probe_depth) {
            max_probe_depth = probe.y;
        }
    }
    
    // Return negative value representing depth below surface
    return -abs(max_probe_depth - min_probe_depth);
}

} // namespace halyard


struct BuoyancyResult {
    Vector3 buoyancy_force = Vector3(0, 0, 0);
    float submerged_ratio = 0.0f; // 0..1
    float submerged_depth = 0.0f; // depth below surface
    Vector3 wave_normal = Vector3(0, 1, 0); // UP
};


inline BuoyancyResult buoyancy_for_probe(
    const Vector3 &probe_global_pos,
    const Transform3D &liquid_surface,
    float full_submerged_depth,
    float density,
    const Vector3 &gravity) {
    BuoyancyResult result;
    
    float depth = liquid_surface.origin.y - probe_global_pos.y; // positive when submerged

    // Calculate submerged ratio (0..1)
    float submerged_ratio = 0.0f;
    if (full_submerged_depth < 0.0f) {
        submerged_ratio = Math::clamp(depth / -full_submerged_depth, 0.0f, 1.0f);
    } else {
        submerged_ratio = (depth > 0.0f) ? 1.0f : 0.0f;
    }
    result.submerged_ratio = submerged_ratio;
    
    if (submerged_ratio > 0.0f) {
        // fade the wave normal towards UP based on submerged ratio
        Vector3 wave_normal = liquid_surface.basis.get_column(1).normalized();
        wave_normal = wave_normal.lerp(Vector3(0, 1, 0), submerged_ratio);

        // Calculate buoyancy force
        Vector3 buoyancy_force = wave_normal * -gravity * density * submerged_ratio;
        result.buoyancy_force = buoyancy_force;
        result.submerged_depth = depth;
    } else {
        result.buoyancy_force = Vector3(0, 0, 0);
        result.submerged_depth = 0.0f;
    }
    
    return result;
}