/* ClipMap

A utility class for generating LOD clipmap meshes for infinite terrain/ocean rendering.
Based on the clipmap algorithm from Zylann's heightmap plugin.

The clipmap generates a series of concentric rings of mesh tiles at increasing LOD levels,
with seam handling to prevent cracks between LOD boundaries.
*/
#pragma once

#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/classes/shader_material.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/core/binder_common.hpp>
#include <godot_cpp/templates/vector.hpp>

using namespace godot;

// Forward declaration
class OceanArea;

class ClipMap : public Resource {
    GDCLASS(ClipMap, Resource)

public:
    // Seam flags for mesh edges that need to connect to coarser LOD
    enum SeamFlags {
        SEAM_NONE = 0,
        SEAM_LEFT = 1,
        SEAM_RIGHT = 2,
        SEAM_BOTTOM = 4,
        SEAM_TOP = 8,
        SEAM_CONFIG_COUNT = 16
    };

private:
    int _chunk_size_x = 16;
    int _chunk_size_y = 16;
    int _lod_count = 3;

    // Mesh cache: _mesh_cache[seam_flags][lod] = Mesh
    Vector<Vector<Ref<ArrayMesh>>> _mesh_cache;

    // Internal mesh generation
    static Ref<ArrayMesh> _make_flat_chunk(int quad_count_x, int quad_count_y, int stride, int seams);
    static PackedInt32Array _make_indices(int chunk_size_x, int chunk_size_y, int seams);

protected:
    static void _bind_methods();

public:
    ClipMap();
    ~ClipMap() override;

    // Configure the clipmap with chunk dimensions and LOD count
    void configure(int chunk_size_x, int chunk_size_y, int lod_count);

    // Get a cached mesh chunk for the specified LOD and seam configuration
    Ref<ArrayMesh> get_chunk(int lod, int seams) const;

    // Getters for configuration
    int get_chunk_size_x() const { return _chunk_size_x; }
    int get_chunk_size_y() const { return _chunk_size_y; }
    int get_lod_count() const { return _lod_count; }

    // Calculate seam flags for a tile position relative to center
    static int flags_for_position(const Vector2 &pos);

    // Utility to get mesh vertex/triangle counts
    static Dictionary get_mesh_size(int width, int height);
};

VARIANT_ENUM_CAST(ClipMap::SeamFlags)


/* ClipMapInstance

A Node3D that manages the visual representation of a clipmap.
Creates and positions mesh instances according to the clipmap configuration.
*/
class ClipMapInstance : public Node3D {
    GDCLASS(ClipMapInstance, Node3D)

private:
    Ref<ClipMap> _clipmap;
    int _tile_size = 32;
    int _lod_levels = 3;
    float _vertex_scaling = 1.0f;

    // Container for tiles
    Node3D *_tiles_container = nullptr;

    // Tile position arrays
    static const Vector2 _twelve_tile_pos[12];
    static const Vector2 _eight_pos[8];
    static const Vector2 _lod0_pos[4];

    void _destroy_tiles();
    void _create_tiles();
    MeshInstance3D *_create_instance(const Ref<Mesh> &mesh, const Vector2 &pos, int level);
    Ref<ShaderMaterial> _get_ocean_material() const;

protected:
    static void _bind_methods();
    void _notification(int p_what);

public:
    ClipMapInstance();
    ~ClipMapInstance() override;
    
    PackedStringArray _get_configuration_warnings() const override;

    // Configuration
    void set_tile_size(int p_size);
    int get_tile_size() const { return _tile_size; }

    void set_lod_levels(int p_levels);
    int get_lod_levels() const { return _lod_levels; }

    void set_vertex_scaling(float p_scale);
    float get_vertex_scaling() const { return _vertex_scaling; }

    // Update the clipmap center position (call each frame with camera/target position)
    void update_position(const Vector3 &target_position);

    // Force rebuild of tile meshes
    void rebuild();
};
