/* ClipMap

A utility class for generating LOD clipmap meshes for infinite terrain/ocean rendering.
*/
#include "clip_map.h"
#include "ocean_area.h"

#include <godot_cpp/classes/plane_mesh.hpp>
#include <godot_cpp/classes/quad_mesh.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/math.hpp>

using namespace godot;

// Static tile position arrays for ClipMapInstance
const Vector2 ClipMapInstance::_twelve_tile_pos[12] = {
	// Top row
	Vector2(-1.5f, 1.5f), Vector2(-0.5f, 1.5f), Vector2(0.5f, 1.5f), Vector2(1.5f, 1.5f),
	// Middle rows (sides only)
	Vector2(-1.5f, 0.5f), Vector2(1.5f, 0.5f),
	Vector2(-1.5f, -0.5f), Vector2(1.5f, -0.5f),
	// Bottom row
	Vector2(-1.5f, -1.5f), Vector2(-0.5f, -1.5f), Vector2(0.5f, -1.5f), Vector2(1.5f, -1.5f)
};

const Vector2 ClipMapInstance::_eight_pos[8] = {
	Vector2(-1.0f, 1.0f), Vector2(0.0f, 1.0f), Vector2(1.0f, 1.0f),
	Vector2(-1.0f, 0.0f), Vector2(1.0f, 0.0f),
	Vector2(-1.0f, -1.0f), Vector2(0.0f, -1.0f), Vector2(1.0f, -1.0f)
};

const Vector2 ClipMapInstance::_lod0_pos[4] = {
	Vector2(-1.0f, -1.0f),
	Vector2(1.0f, -1.0f),
	Vector2(1.0f, 1.0f),
	Vector2(-1.0f, 1.0f)
};

//------------------------------------------------------------------------------
// ClipMap
//------------------------------------------------------------------------------

ClipMap::ClipMap() {
}

ClipMap::~ClipMap() {
}

void ClipMap::_bind_methods() {
	ClassDB::bind_method(D_METHOD("configure", "chunk_size_x", "chunk_size_y", "lod_count"), &ClipMap::configure);
	ClassDB::bind_method(D_METHOD("get_chunk", "lod", "seams"), &ClipMap::get_chunk);
	ClassDB::bind_method(D_METHOD("get_chunk_size_x"), &ClipMap::get_chunk_size_x);
	ClassDB::bind_method(D_METHOD("get_chunk_size_y"), &ClipMap::get_chunk_size_y);
	ClassDB::bind_method(D_METHOD("get_lod_count"), &ClipMap::get_lod_count);
	ClassDB::bind_static_method("ClipMap", D_METHOD("flags_for_position", "pos"), &ClipMap::flags_for_position);
	ClassDB::bind_static_method("ClipMap", D_METHOD("get_mesh_size", "width", "height"), &ClipMap::get_mesh_size);

	BIND_ENUM_CONSTANT(SEAM_NONE);
	BIND_ENUM_CONSTANT(SEAM_LEFT);
	BIND_ENUM_CONSTANT(SEAM_RIGHT);
	BIND_ENUM_CONSTANT(SEAM_BOTTOM);
	BIND_ENUM_CONSTANT(SEAM_TOP);
	BIND_ENUM_CONSTANT(SEAM_CONFIG_COUNT);
}

void ClipMap::configure(int chunk_size_x, int chunk_size_y, int lod_count) {
	ERR_FAIL_COND(chunk_size_x < 2 || chunk_size_y < 2);
	ERR_FAIL_COND(lod_count < 1);

	// Check if already configured with same parameters
	if (chunk_size_x == _chunk_size_x && chunk_size_y == _chunk_size_y &&
			lod_count == _lod_count && _mesh_cache.size() == SEAM_CONFIG_COUNT) {
		return;
	}

	_chunk_size_x = chunk_size_x;
	_chunk_size_y = chunk_size_y;
	_lod_count = lod_count;

	_mesh_cache.resize(SEAM_CONFIG_COUNT);

	for (int seams = 0; seams < SEAM_CONFIG_COUNT; seams++) {
		_mesh_cache.write[seams].resize(lod_count);

		for (int lod = 0; lod < lod_count; lod++) {
			int stride = 1 << lod;
			_mesh_cache.write[seams].write[lod] = _make_flat_chunk(_chunk_size_x, _chunk_size_y, stride, seams);
		}
	}
}

Ref<ArrayMesh> ClipMap::get_chunk(int lod, int seams) const {
	ERR_FAIL_INDEX_V(seams, SEAM_CONFIG_COUNT, Ref<ArrayMesh>());
	ERR_FAIL_INDEX_V(lod, _lod_count, Ref<ArrayMesh>());

	if (_mesh_cache.size() == 0) {
		return Ref<ArrayMesh>();
	}

	return _mesh_cache[seams][lod];
}

int ClipMap::flags_for_position(const Vector2 &pos) {
	int flags = 0;
	if (pos.x > 0.5f)
		flags |= SEAM_RIGHT;
	if (pos.x < -0.5f)
		flags |= SEAM_LEFT;
	if (pos.y > 0.5f)
		flags |= SEAM_TOP;
	if (pos.y < -0.5f)
		flags |= SEAM_BOTTOM;
	return flags;
}

Dictionary ClipMap::get_mesh_size(int width, int height) {
	Dictionary result;
	result["vertices"] = width * height;
	result["triangles"] = (width - 1) * (height - 1) * 2;
	return result;
}

Ref<ArrayMesh> ClipMap::_make_flat_chunk(int quad_count_x, int quad_count_y, int stride, int seams) {
	PackedVector3Array positions;
	positions.resize((quad_count_x + 1) * (quad_count_y + 1));

	Vector3 *pos_ptr = positions.ptrw();
	int i = 0;
	for (int y = 0; y <= quad_count_y; y++) {
		for (int x = 0; x <= quad_count_x; x++) {
			pos_ptr[i] = Vector3(static_cast<float>(x * stride), 0.0f, static_cast<float>(y * stride));
			i++;
		}
	}

	PackedInt32Array indices = _make_indices(quad_count_x, quad_count_y, seams);

	PackedVector3Array normals;
	normals.resize(positions.size());
	Vector3 *norm_ptr = normals.ptrw();
	for (int j = 0; j < normals.size(); j++) {
		norm_ptr[j] = Vector3(0.0f, 1.0f, 0.0f);
	}

	Array arrays;
	arrays.resize(Mesh::ARRAY_MAX);
	arrays[Mesh::ARRAY_VERTEX] = positions;
	arrays[Mesh::ARRAY_INDEX] = indices;
	arrays[Mesh::ARRAY_NORMAL] = normals;

	Ref<ArrayMesh> mesh;
	mesh.instantiate();
	mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);

	return mesh;
}

PackedInt32Array ClipMap::_make_indices(int chunk_size_x, int chunk_size_y, int seams) {
	PackedInt32Array output_indices;

	if (seams != 0) {
		// LOD seams can't be made properly on uneven chunk sizes
		ERR_FAIL_COND_V(chunk_size_x % 2 != 0 || chunk_size_y % 2 != 0, output_indices);
	}

	int reg_origin_x = 0;
	int reg_origin_y = 0;
	int reg_size_x = chunk_size_x;
	int reg_size_y = chunk_size_y;
	int reg_hstride = 1;

	if (seams & SEAM_LEFT) {
		reg_origin_x += 1;
		reg_size_x -= 1;
		reg_hstride += 1;
	}

	if (seams & SEAM_BOTTOM) {
		reg_origin_y += 1;
		reg_size_y -= 1;
	}

	if (seams & SEAM_RIGHT) {
		reg_size_x -= 1;
		reg_hstride += 1;
	}

	if (seams & SEAM_TOP) {
		reg_size_y -= 1;
	}

	// Regular triangles
	int ii = reg_origin_x + reg_origin_y * (chunk_size_x + 1);

	for (int y = 0; y < reg_size_y; y++) {
		for (int x = 0; x < reg_size_x; x++) {
			int i00 = ii;
			int i10 = ii + 1;
			int i01 = ii + chunk_size_x + 1;
			int i11 = i01 + 1;

			// Flip pattern for orientation-free geometry
			bool flip = (((x + reg_origin_x) + (y + reg_origin_y)) % 2) != 0;

			if (flip) {
				output_indices.push_back(i00);
				output_indices.push_back(i10);
				output_indices.push_back(i01);

				output_indices.push_back(i10);
				output_indices.push_back(i11);
				output_indices.push_back(i01);
			} else {
				output_indices.push_back(i00);
				output_indices.push_back(i11);
				output_indices.push_back(i01);

				output_indices.push_back(i00);
				output_indices.push_back(i10);
				output_indices.push_back(i11);
			}

			ii += 1;
		}
		ii += reg_hstride;
	}

	// Left seam
	if (seams & SEAM_LEFT) {
		int i = 0;
		int n = chunk_size_y / 2;

		for (int j = 0; j < n; j++) {
			int i0 = i;
			int i1 = i + 1;
			int i3 = i + chunk_size_x + 2;
			int i4 = i + 2 * (chunk_size_x + 1);
			int i5 = i4 + 1;

			output_indices.push_back(i0);
			output_indices.push_back(i3);
			output_indices.push_back(i4);

			if (j != 0 || (seams & SEAM_BOTTOM) == 0) {
				output_indices.push_back(i0);
				output_indices.push_back(i1);
				output_indices.push_back(i3);
			}

			if (j != n - 1 || (seams & SEAM_TOP) == 0) {
				output_indices.push_back(i3);
				output_indices.push_back(i5);
				output_indices.push_back(i4);
			}

			i = i4;
		}
	}

	// Right seam
	if (seams & SEAM_RIGHT) {
		int i = chunk_size_x - 1;
		int n = chunk_size_y / 2;

		for (int j = 0; j < n; j++) {
			int i0 = i;
			int i1 = i + 1;
			int i2 = i + chunk_size_x + 1;
			int i4 = i + 2 * (chunk_size_x + 1);
			int i5 = i4 + 1;

			output_indices.push_back(i1);
			output_indices.push_back(i5);
			output_indices.push_back(i2);

			if (j != 0 || (seams & SEAM_BOTTOM) == 0) {
				output_indices.push_back(i0);
				output_indices.push_back(i1);
				output_indices.push_back(i2);
			}

			if (j != n - 1 || (seams & SEAM_TOP) == 0) {
				output_indices.push_back(i2);
				output_indices.push_back(i5);
				output_indices.push_back(i4);
			}

			i = i4;
		}
	}

	// Bottom seam
	if (seams & SEAM_BOTTOM) {
		int i = 0;
		int n = chunk_size_x / 2;

		for (int j = 0; j < n; j++) {
			int i0 = i;
			int i2 = i + 2;
			int i3 = i + chunk_size_x + 1;
			int i4 = i3 + 1;
			int i5 = i4 + 1;

			output_indices.push_back(i0);
			output_indices.push_back(i2);
			output_indices.push_back(i4);

			if (j != 0 || (seams & SEAM_LEFT) == 0) {
				output_indices.push_back(i0);
				output_indices.push_back(i4);
				output_indices.push_back(i3);
			}

			if (j != n - 1 || (seams & SEAM_RIGHT) == 0) {
				output_indices.push_back(i2);
				output_indices.push_back(i5);
				output_indices.push_back(i4);
			}

			i = i2;
		}
	}

	// Top seam
	if (seams & SEAM_TOP) {
		int i = (chunk_size_y - 1) * (chunk_size_x + 1);
		int n = chunk_size_x / 2;

		for (int j = 0; j < n; j++) {
			int i0 = i;
			int i1 = i + 1;
			int i2 = i + 2;
			int i3 = i + chunk_size_x + 1;
			int i5 = i3 + 2;

			output_indices.push_back(i3);
			output_indices.push_back(i1);
			output_indices.push_back(i5);

			if (j != 0 || (seams & SEAM_LEFT) == 0) {
				output_indices.push_back(i0);
				output_indices.push_back(i1);
				output_indices.push_back(i3);
			}

			if (j != n - 1 || (seams & SEAM_RIGHT) == 0) {
				output_indices.push_back(i1);
				output_indices.push_back(i2);
				output_indices.push_back(i5);
			}

			i = i2;
		}
	}

	return output_indices;
}

//------------------------------------------------------------------------------
// ClipMapInstance
//------------------------------------------------------------------------------

ClipMapInstance::ClipMapInstance() {
}

ClipMapInstance::~ClipMapInstance() {
}

void ClipMapInstance::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_tile_size", "size"), &ClipMapInstance::set_tile_size);
	ClassDB::bind_method(D_METHOD("get_tile_size"), &ClipMapInstance::get_tile_size);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "tile_size", PROPERTY_HINT_RANGE, "2,256,1"), "set_tile_size", "get_tile_size");

	ClassDB::bind_method(D_METHOD("set_lod_levels", "levels"), &ClipMapInstance::set_lod_levels);
	ClassDB::bind_method(D_METHOD("get_lod_levels"), &ClipMapInstance::get_lod_levels);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "lod_levels", PROPERTY_HINT_RANGE, "0,10,1"), "set_lod_levels", "get_lod_levels");

	ClassDB::bind_method(D_METHOD("set_vertex_scaling", "scale"), &ClipMapInstance::set_vertex_scaling);
	ClassDB::bind_method(D_METHOD("get_vertex_scaling"), &ClipMapInstance::get_vertex_scaling);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "vertex_scaling", PROPERTY_HINT_RANGE, "0.03125,32,0.03125"), "set_vertex_scaling", "get_vertex_scaling");

	ClassDB::bind_method(D_METHOD("update_position", "target_position"), &ClipMapInstance::update_position);
	ClassDB::bind_method(D_METHOD("rebuild"), &ClipMapInstance::rebuild);
}

void ClipMapInstance::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE:
			_create_tiles();
			break;

		case NOTIFICATION_READY:
			break;

		case NOTIFICATION_EXIT_TREE:
			_destroy_tiles();
			break;
	}
}

void ClipMapInstance::set_tile_size(int p_size) {
	if (p_size < 2)
		p_size = 2;
	if (p_size % 2 != 0)
		p_size += 1; // Must be even for seams
	if (_tile_size != p_size) {
		_tile_size = p_size;
		_create_tiles();
	}
}

void ClipMapInstance::set_lod_levels(int p_levels) {
	if (p_levels < 0)
		p_levels = 0;
	if (_lod_levels != p_levels) {
		_lod_levels = p_levels;
		_create_tiles();
	}
}

void ClipMapInstance::set_vertex_scaling(float p_scale) {
	_vertex_scaling = p_scale;
	if (_tiles_container) {
		_tiles_container->set_scale(Vector3(_vertex_scaling, _vertex_scaling, _vertex_scaling));
	}
}

Ref<ShaderMaterial> ClipMapInstance::_get_ocean_material() const {
	// Get material from parent Ocean node
	Node *parent = get_parent();
	if (parent) {
		OceanArea *ocean = Object::cast_to<OceanArea>(parent);
		if (ocean) {
			Ref<WaveSampler> wave_sampler = ocean->get_wave_sampler();
			if (wave_sampler.is_valid()) {
				return wave_sampler->get_material();
			}
		}
	}
	return Ref<ShaderMaterial>();
}

PackedStringArray ClipMapInstance::_get_configuration_warnings() const {
	PackedStringArray warnings;

	Node *parent = get_parent();
	if (!parent || !Object::cast_to<OceanArea>(parent)) {
		warnings.push_back("ClipMapInstance should be a child of an Ocean node.");
	}

	return warnings;
}

void ClipMapInstance::update_position(const Vector3 &target_position) {
	set_global_position(target_position);
	// if (_tiles_container) {
	//     _tiles_container->set_global_position(Vector3(target_position.x, 0, target_position.z));
	// }
}

void ClipMapInstance::rebuild() {
	_create_tiles();
}

void ClipMapInstance::_destroy_tiles() {
	if (_tiles_container) {
		_tiles_container->queue_free();
		_tiles_container = nullptr;
	}
	_clipmap.unref();
}

MeshInstance3D *ClipMapInstance::_create_instance(const Ref<Mesh> &mesh, const Vector2 &pos, int level) {
	if (mesh.is_null() || !_tiles_container) {
		return nullptr;
	}

	MeshInstance3D *instance = memnew(MeshInstance3D);
	_tiles_container->add_child(instance);

	instance->set_extra_cull_margin(static_cast<float>(level + 1));
	instance->set_mesh(mesh);
	instance->set_surface_override_material(0, _get_ocean_material());

	instance->set_position(Vector3(pos.x, 0.0f, pos.y));

	return instance;
}

void ClipMapInstance::_create_tiles() {
	_destroy_tiles();

	if (!is_inside_tree()) {
		return;
	}

	// Create clipmap mesh cache
	_clipmap.instantiate();
	_clipmap->configure(_tile_size, _tile_size, MAX(_lod_levels, 1));

	// Container node
	_tiles_container = memnew(Node3D);
	_tiles_container->set_scale(Vector3(_vertex_scaling, _vertex_scaling, _vertex_scaling));
	add_child(_tiles_container);

	Vector2 half_size = Vector2(static_cast<float>(_tile_size), static_cast<float>(_tile_size)) * 0.5f;

	// Create central 4-up LOD0 tiles
	for (int i = 0; i < 4; i++) {
		Ref<ArrayMesh> lod0_mesh = _clipmap->get_chunk(0, ClipMap::SEAM_NONE);
		Vector2 pos = half_size * _lod0_pos[i] - half_size;
		_create_instance(lod0_mesh, pos, 0);
	}

	// Create subsequent rings for LOD1..n
	for (int r = 0; r < _lod_levels; r++) {
		float lod_size = static_cast<float>(_tile_size * (1 << r));

		for (int n = 0; n < 12; n++) {
			Vector2 tile_pos = _twelve_tile_pos[n];
			int seam_flags = ClipMap::flags_for_position(tile_pos);
			Ref<ArrayMesh> lodN_mesh = _clipmap->get_chunk(r, seam_flags);
			Vector2 pos = lod_size * tile_pos - Vector2(lod_size * 0.5f, lod_size * 0.5f);
			_create_instance(lodN_mesh, pos, r);
		}
	}

	// Final ring - simple quads to absorb the horizon
	Ref<QuadMesh> last_shape;
	last_shape.instantiate();
	last_shape->set_orientation(PlaneMesh::FACE_Y);

	float last_size = static_cast<float>(_tile_size * (1 << _lod_levels) + _tile_size * (1 << (_lod_levels > 0 ? _lod_levels - 1 : 0)) * 2);
	last_shape->set_size(Vector2(last_size, last_size));

	for (int i = 0; i < 8; i++) {
		Vector2 pos = last_size * _eight_pos[i];
		MeshInstance3D *instance = _create_instance(last_shape, pos, 0);
		if (instance) {
			// Drop by 1 meter to hide border at extreme distances
			instance->set_position(Vector3(pos.x, -1.0f, pos.y));
		}
	}
}
