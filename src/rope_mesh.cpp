/*
 * Copyright (c) 2026 M. Estee.
 * Licensed under the MIT License.
 */

#include "rope_mesh.h"

#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/classes/mesh.hpp>
#include <godot_cpp/classes/primitive_mesh.hpp>
#include <godot_cpp/core/math.hpp>

void RopeMesh::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_sides", "sides"), &RopeMesh::set_sides);
	ClassDB::bind_method(D_METHOD("get_sides"), &RopeMesh::get_sides);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "sides", PROPERTY_HINT_RANGE, "0,128,1,or_greater"), "set_sides", "get_sides");

	ClassDB::bind_method(D_METHOD("set_radius", "radius"), &RopeMesh::set_radius);
	ClassDB::bind_method(D_METHOD("get_radius"), &RopeMesh::get_radius);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "radius", PROPERTY_HINT_RANGE, "0,10,0.001,or_greater"), "set_radius", "get_radius");

	ClassDB::bind_method(D_METHOD("set_rope_length", "rope_length"), &RopeMesh::set_rope_length);
	ClassDB::bind_method(D_METHOD("get_rope_length"), &RopeMesh::get_rope_length);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "rope_length", PROPERTY_HINT_RANGE, "0,1000,0.01,or_greater"), "set_rope_length", "get_rope_length");

	ClassDB::bind_method(D_METHOD("set_rope_width", "rope_width"), &RopeMesh::set_rope_width);
	ClassDB::bind_method(D_METHOD("get_rope_width"), &RopeMesh::get_rope_width);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "rope_width", PROPERTY_HINT_RANGE, "0,10,0.001,or_greater"), "set_rope_width", "get_rope_width");

	ClassDB::bind_method(D_METHOD("set_rope_twist", "rope_twist"), &RopeMesh::set_rope_twist);
	ClassDB::bind_method(D_METHOD("get_rope_twist"), &RopeMesh::get_rope_twist);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "rope_twist", PROPERTY_HINT_RANGE, "0,10,0.01,or_greater"), "set_rope_twist", "get_rope_twist");
}

void RopeMesh::set_sides(int p_sides) {
	_sides = Math::max(p_sides, 0);
}

int RopeMesh::get_sides() const {
	return _sides;
}

void RopeMesh::set_radius(float p_radius) {
	_radius = Math::max(p_radius, 0.0f);
}

float RopeMesh::get_radius() const {
	return _radius;
}

void RopeMesh::set_rope_length(float p_rope_length) {
	_rope_length = Math::max(p_rope_length, 0.0f);
}

float RopeMesh::get_rope_length() const {
	return _rope_length;
}

void RopeMesh::set_rope_width(float p_rope_width) {
	_rope_width = Math::max(p_rope_width, 0.0f);
}

float RopeMesh::get_rope_width() const {
	return _rope_width;
}

void RopeMesh::set_rope_twist(float p_rope_twist) {
	_rope_twist = p_rope_twist;
}

float RopeMesh::get_rope_twist() const {
	return _rope_twist;
}

void RopeMesh::clear_mesh() {
	_sides = 0;
	_radius = 0.0f;
	_rope_length = 0.0f;
	_rope_width = 0.0f;
	_rope_twist = 1.0f;

	_verts.clear();
	_norms.clear();
	_uv1s.clear();
	_cum_lengths.clear();

	clear_surfaces();
}

#define X 0
#define Y 1
#define Z 2

void RopeMesh::_emit_tube(const LocalVector<Transform3D> &p_frames, PackedVector3Array &p_V, PackedVector3Array &p_N, PackedVector2Array &p_UV1) const {
	// build cumulative length along the sampled positions so we can map V smoothly.
	// rope can be stretchy so we can't just use rope_length here
	// Reuse cached vector to avoid allocation
	_cum_lengths.clear();
	_cum_lengths.push_back(0.0);
	for (int k = 1; k < p_frames.size(); k++)
		_cum_lengths.push_back(_cum_lengths[k - 1] + p_frames[k - 1].origin.distance_to(p_frames[k].origin));

	float total_length = _cum_lengths[_cum_lengths.size() - 1];
	if (total_length <= 0.0)
		total_length = 1.0;

	// number of V repeats along the rope is based on rope width and twist factor
	const float repeats = get_rope_length() / get_rope_width() * get_rope_twist();
	float inv_total_length = 1.0f / total_length;
	float inv_sides = 1.0f / float(_sides);

	// NOTE: run to the second to the last frame as we emit 2 frames at a time.
	for (int i = 0; i < p_frames.size() - 1; i++) {
		const auto &pos = p_frames[i].origin;
		const auto &next_pos = p_frames[i + 1].origin;

		const auto &norm = p_frames[i].basis.get_column(X);
		const auto &binorm = p_frames[i].basis.get_column(Z);
		const auto &next_norm = p_frames[i + 1].basis.get_column(X);
		const auto &next_binorm = p_frames[i + 1].basis.get_column(Z);

		const auto v = (_cum_lengths[i] * inv_total_length) * repeats;
		const auto next_v = (_cum_lengths[i + 1] * inv_total_length) * repeats;

		// expand AABB using frame origins + radius
		_aabb.expand_to(pos);
		_aabb.expand_to(next_pos);

		// loop one extra to close the seam (repeat first vertex)
		// for the first and the last row.
		for (int j = _sides; j >= 0; j--) {
			const auto wrap_j = j % _sides;
			const auto angle = Math_TAU * float(wrap_j) * inv_sides;
			const auto ca = cos(angle);
			const auto sa = sin(angle);

			const auto offset = (binorm * ca + norm * sa) * _radius;
			const auto next_offset = (next_binorm * ca + next_norm * sa) * _radius;

			const auto normal = offset.normalized();
			const auto next_normal = next_offset.normalized();

			// U goes 0..1 around the tube; use j so the final seam vertex reaches 1.0;
			const auto u = float(j) * inv_sides;

			p_V.push_back(pos + offset);
			p_N.push_back(normal);
			p_UV1.push_back(Vector2(u, v));

			p_V.push_back(next_pos + next_offset);
			p_N.push_back(next_normal);
			p_UV1.push_back(Vector2(u, next_v));
		}
	}
}

void RopeMesh::_emit_endcap(bool p_front, const Transform3D &p_frame, PackedVector3Array &p_V, PackedVector3Array &p_N, PackedVector2Array &p_UV1) const {
	Vector3 center = p_frame.origin;
	Vector3 T = p_frame.basis.get_column(Y);
	Vector3 N = p_frame.basis.get_column(X);
	Vector3 B = p_frame.basis.get_column(Z);

	// expand AABB for endcap center
	_aabb.expand_to(center);

	// UV to be aligned radially around the rope edge as a function of the side count
	float u_width = 1.0 / _sides;
	Vector3 center_normal = T.normalized() * (p_front ? -1.0 : 1.0);

	auto emit = [&](int j) {
		const auto wrap_j = j % _sides;
		const auto angle = Math_TAU * float(wrap_j) / float(_sides);
		const auto ca = cos(angle);
		const auto sa = sin(angle);
		const auto a = center + (B * ca + N * sa) * _radius;
		const auto uv_a = Vector2(wrap_j * u_width, 0);

		const auto center_uv = Vector2(j * u_width, 0);

		p_V.push_back(a);
		p_N.push_back(center_normal);
		p_UV1.push_back(uv_a);

		p_V.push_back(center);
		p_N.push_back(center_normal);
		p_UV1.push_back(center_uv);
	};

	// emit triangles for end cap in either CCW or CW order
	if (p_front) {
		for (int j = 0; j < _sides + 1; j++)
			emit(j);
	} else {
		for (int j = _sides; j >= 0; j--) {
			emit(j);
		}
	}
}

void RopeMesh::_update_mesh_internal(const LocalVector<Transform3D> &p_frames, Ref<Material> p_material) {
	Array mesh;
	mesh.resize(Mesh::ARRAY_MAX);

	// failure case, return a single vertex to avoid issues with zero-vertex meshes
	bool has_frames = p_frames.size() >= 2 && _sides >= 3 && _radius > 0.0f;
	if (has_frames == false) {
		PackedVector3Array verts;
		// emit a tiny degenerate triangle to satisfy primitive minimum vertex count
		verts.push_back(Vector3());
		verts.push_back(Vector3(0.001f, 0.0f, 0.0f));
		verts.push_back(Vector3(0.0f, 0.001f, 0.0f));

		PackedVector3Array norms;
		norms.push_back(Vector3(0, 1, 0));
		norms.push_back(Vector3(0, 1, 0));
		norms.push_back(Vector3(0, 1, 0));

		mesh[Mesh::ARRAY_VERTEX] = verts;
		mesh[Mesh::ARRAY_NORMAL] = norms;
	} else {
		_verts.clear();
		_norms.clear();
		_uv1s.clear();

		// reset AABB before accumulating
		_aabb = AABB();

		_emit_endcap(true, p_frames[0], _verts, _norms, _uv1s);
		_emit_tube(p_frames, _verts, _norms, _uv1s);
		_emit_endcap(false, p_frames[p_frames.size() - 1], _verts, _norms, _uv1s);

		// pad AABB by rope radius to fully enclose mesh
		_aabb = _aabb.grow(_radius);
		const_cast<RopeMesh *>(this)->set_custom_aabb(_aabb);

		mesh[Mesh::ARRAY_VERTEX] = _verts;
		mesh[Mesh::ARRAY_NORMAL] = _norms;
		mesh[Mesh::ARRAY_TEX_UV] = _uv1s;
	}

	clear_surfaces();
	add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLE_STRIP, mesh);
	surface_set_material(0, p_material);
	set_custom_aabb(_aabb);
}