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
	ADD_PROPERTY(PropertyInfo(Variant::INT, "sides", PROPERTY_HINT_RANGE, "3,128,1,or_greater"), "set_sides", "get_sides");

	ClassDB::bind_method(D_METHOD("set_radius", "radius"), &RopeMesh::set_radius);
	ClassDB::bind_method(D_METHOD("get_radius"), &RopeMesh::get_radius);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "radius", PROPERTY_HINT_RANGE, "0.001,10,0.001,or_greater"), "set_radius", "get_radius");

	ClassDB::bind_method(D_METHOD("set_rope_length", "rope_length"), &RopeMesh::set_rope_length);
	ClassDB::bind_method(D_METHOD("get_rope_length"), &RopeMesh::get_rope_length);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "rope_length", PROPERTY_HINT_RANGE, "0.001,1000,0.001,or_greater"), "set_rope_length", "get_rope_length");

	ClassDB::bind_method(D_METHOD("set_rope_twist", "rope_twist"), &RopeMesh::set_rope_twist);
	ClassDB::bind_method(D_METHOD("get_rope_twist"), &RopeMesh::get_rope_twist);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "rope_twist", PROPERTY_HINT_RANGE, "0,10,0.01,or_greater"), "set_rope_twist", "get_rope_twist");

	ClassDB::bind_method(D_METHOD("clear_mesh"), &RopeMesh::clear_mesh);

	// Mesh gen
	ClassDB::bind_method(D_METHOD("begin_update_mesh"), &RopeMesh::begin_update_mesh);
	ClassDB::bind_method(D_METHOD("emit_tube", "frames"), &RopeMesh::emit_tube_bind);
	ClassDB::bind_method(D_METHOD("emit_endcap", "front", "frame"), &RopeMesh::emit_endcap);
	ClassDB::bind_method(D_METHOD("end_update_mesh"), &RopeMesh::end_update_mesh);
}

void RopeMesh::set_sides(int p_sides) {
	_sides = Math::max(p_sides, 3);
}

int RopeMesh::get_sides() const {
	return _sides;
}

void RopeMesh::set_radius(float p_radius) {
	_radius = Math::max(p_radius, 0.001f);
}

float RopeMesh::get_radius() const {
	return _radius;
}

void RopeMesh::set_rope_length(float p_rope_length) {
	_rope_length = Math::max(p_rope_length, 0.001f);
}

float RopeMesh::get_rope_length() const {
	return _rope_length;
}

void RopeMesh::set_rope_twist(float p_rope_twist) {
	_rope_twist = p_rope_twist;
}

float RopeMesh::get_rope_twist() const {
	return _rope_twist;
}

void RopeMesh::clear_mesh() {
	set_custom_aabb(AABB());
	clear_surfaces();
}

#define X 0
#define Y 1
#define Z 2

void RopeMesh::begin_update_mesh() {
	_verts.clear();
	_norms.clear();
	_uv1s.clear();
	_cum_lengths.clear();
	_aabb = AABB();
}


void RopeMesh::end_update_mesh(Ref<Material> p_material) {
	// build the mesh from the generated vertex data
	if ( _verts.size() > 0 && _norms.size() > 0 && _uv1s.size() > 0) {
		Array arrays;
		arrays.resize(Mesh::ARRAY_MAX);
		arrays[Mesh::ARRAY_VERTEX] = _verts;
		arrays[Mesh::ARRAY_NORMAL] = _norms;
		arrays[Mesh::ARRAY_TEX_UV] = _uv1s;

		// pad AABB by rope radius to fully enclose mesh
		_aabb = _aabb.grow(_radius);

		// include existing AABB
		_aabb = _aabb.merge(get_custom_aabb());
		set_custom_aabb(_aabb);

		add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLE_STRIP, arrays);
		surface_set_material(get_surface_count() - 1, p_material);
	}

	// cleanup
	_verts.clear();
	_norms.clear();
	_uv1s.clear();
	_cum_lengths.clear();
	_aabb = AABB();
}


void RopeMesh::emit_tube(const LocalVector<Transform3D> &p_frames) {
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
	const float repeats = get_rope_length() / (get_radius() * 2.0) * get_rope_twist();
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

			_verts.push_back(pos + offset);
			_norms.push_back(normal);
			_uv1s.push_back(Vector2(u, v));

			_verts.push_back(next_pos + next_offset);
			_norms.push_back(next_normal);
			_uv1s.push_back(Vector2(u, next_v));
		}
	}
}

void RopeMesh::emit_tube_bind(const TypedArray<Transform3D> &p_frames){
	LocalVector<Transform3D> frames;
	frames.resize(p_frames.size());
	for (int i = 0; i < p_frames.size(); i++)
		frames[i] = p_frames[i];

	emit_tube(frames);
}

void RopeMesh::emit_endcap(bool p_front, const Transform3D &p_frame) {
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

		_verts.push_back(a);
		_norms.push_back(center_normal);
		_uv1s.push_back(uv_a);

		_verts.push_back(center);
		_norms.push_back(center_normal);
		_uv1s.push_back(center_uv);
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

