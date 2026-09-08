/*
 * Copyright (c) 2026 M. Estee.
 * Licensed under the MIT License.
 */

#pragma once

#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/core/gdvirtual.gen.inc>
#include <godot_cpp/templates/local_vector.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/packed_vector2_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/typed_array.hpp>
#include <godot_cpp/variant/variant.hpp>
#include <godot_cpp/variant/vector2.hpp>

using namespace godot;

class RopeMesh : public ArrayMesh {
	GDCLASS(RopeMesh, ArrayMesh)

	// LocalVector<Transform3D> _frames;
	int _sides = 3;
	float _radius = 1.0f;
	float _rope_length = 1.0f;
	float _rope_twist = 1.0f;

	// temp buffers used for begin/end update
	mutable PackedVector3Array _verts;
	mutable PackedVector3Array _norms;
	mutable PackedVector2Array _uv1s;
	mutable LocalVector<float> _cum_lengths;
	mutable AABB _aabb;

protected:
	static void _bind_methods();

public:
	RopeMesh() = default;
	virtual ~RopeMesh() override = default;

	void set_sides(int p_sides);
	int get_sides() const;

	void set_radius(float p_radius);
	float get_radius() const;

	void set_rope_length(float p_rope_length);
	float get_rope_length() const;

	void set_rope_twist(float p_rope_twist);
	float get_rope_twist() const;

	void begin_update_mesh();
	void emit_tube(const LocalVector<Transform3D> &p_frames);
	void emit_tube_bind(const TypedArray<Transform3D> &p_frames);
	void emit_endcap(bool p_front, const Transform3D &p_frame);
	void end_update_mesh(Ref<Material> p_material);

	void clear_mesh();
};
