#pragma once

#include <godot_cpp/variant/packed_vector3_array.hpp>

using namespace godot;

namespace halyard {

/// Calculate the full submerged depth based on probe extents in local space
/// This represents the depth at which all probes would be fully submerged
/// Returns a negative value (depth below surface)
inline float calculate_full_submerged_depth(const godot::PackedVector3Array &probes, const Transform3D &global_xform) {
    if (probes.size() == 0) {
        return 0.0f;
    }
    
    float min_probe_depth = 0.0f;
    float max_probe_depth = 0.0f;
    
    for (int i = 0; i < probes.size(); i++) {
        const godot::Vector3 &probe = probes[i];
        Vector3 global_probe = global_xform.xform(probe);
        if (global_probe.y < min_probe_depth) {
            min_probe_depth = global_probe.y;
        }
        if (global_probe.y > max_probe_depth) {
            max_probe_depth = global_probe.y;
        }
    }
    
    // Return negative value representing depth below surface
    return -abs(max_probe_depth - min_probe_depth);
}

} // namespace halyard