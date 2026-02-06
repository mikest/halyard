/* MeshBuoyancy
 *
 * A utility class for calculating buoyancy forces from a submerged mesh volume.
 *
 * This class encapsulates mesh-based buoyancy calculations using triangle
 * clipping against a liquid surface to determine submerged volume and centroid.
 *
 * Copyright (c) M. Estee
 * MIT License.
 */

#include "mesh_buoyancy.h"
#include "liquid_area.h"

using namespace godot;
using namespace halyard;

#pragma region Accessors

void MeshBuoyancy::set_liquid_area(LiquidArea *p_liquid_area) {
	_liquid_area = p_liquid_area;
}

LiquidArea *MeshBuoyancy::get_liquid_area() const {
	return _liquid_area;
}

void MeshBuoyancy::set_buoyancy_mesh(const Ref<ArrayMesh> &p_mesh) {
	_buoyancy_mesh = p_mesh;
}

Ref<ArrayMesh> MeshBuoyancy::get_buoyancy_mesh() const {
	return _buoyancy_mesh;
}

void MeshBuoyancy::set_ignore_waves(bool p_ignore) {
	_ignore_waves = p_ignore;
}

bool MeshBuoyancy::get_ignore_waves() const {
	return _ignore_waves;
}

void MeshBuoyancy::set_buoyancy(float p_buoyancy) {
	_buoyancy = p_buoyancy;
}

float MeshBuoyancy::get_buoyancy() const {
	return _buoyancy;
}

void MeshBuoyancy::set_mass(float p_mass) {
	_mass = p_mass;
}

float MeshBuoyancy::get_mass() const {
	return _mass;
}

float MeshBuoyancy::get_mesh_volume() const {
	return _mesh_volume;
}

Vector3 MeshBuoyancy::get_mesh_centroid() const {
	return _mesh_centroid;
}

const PackedVector3Array &MeshBuoyancy::get_vertices() const {
	return _vertex;
}

float MeshBuoyancy::get_submerged_volume() const {
	return _submerged_volume;
}

Vector3 MeshBuoyancy::get_submerged_centroid() const {
	return _submerged_centroid;
}

Vector3 MeshBuoyancy::get_buoyancy_normal() const {
	return _buoyancy_normal;
}

float MeshBuoyancy::get_submerged_ratio() const {
	if (Math::is_zero_approx(_mesh_volume)) {
		return 0.0f;
	}
	return _submerged_volume / _mesh_volume;
}

Vector3 MeshBuoyancy::get_buoyancy_force() const {
	return _buoyancy_force;
}

Vector3 MeshBuoyancy::get_force_position() const {
	return _force_position;
}

const PackedVector3Array &MeshBuoyancy::get_submerged_verts() const {
	return _submerged_verts;
}

#pragma endregion

#pragma region Triangle Calculations

Vector4 MeshBuoyancy::_tri_contribution(const Vector3 &a, const Vector3 &b, const Vector3 &c, const Vector3 &o) const {
	float V = _sign * (a - o).cross(b - o).dot(c - o) / 6.0f;
	Vector3 C = V * (a + b + c + o) / 4.0f;
	return Vector4(C.x, C.y, C.z, V);
}

Vector3 MeshBuoyancy::_intersect(const Vector3 &v1, const Vector3 &v2, float d1, float d2) const {
	if (Math::is_equal_approx(d1, d2)) {
		return v1;
	}
	float t = d1 / (d1 - d2);
	return v1.lerp(v2, t);
}

// Calculates the submerged volume and centroid contribution of a triangle
// given the depths of its vertices. Handles partial submersion by clipping
// the triangle against the liquid plane.
//
// Also collects submerged vertices for debug visualization.
Vector4 MeshBuoyancy::_partial_intersection(const Vector3 &a, const Vector3 &b, const Vector3 &c, const Vector3 &o,
		float _a, float _b, float _c, bool keep_below) {
	Vector3 C = Vector3(0, 0, 0);
	float V = 0.0f;

	bool below_a = _a <= 0.0f;
	bool below_b = _b <= 0.0f;
	bool below_c = _c <= 0.0f;
	if (!keep_below) {
		below_a = !below_a;
		below_b = !below_b;
		below_c = !below_c;
	}
	int count = (int)below_a + (int)below_b + (int)below_c;

	// Trivial cases
	if (count == 3) {
		_submerged_verts.append(a);
		_submerged_verts.append(b);
		_submerged_verts.append(c);
		return _tri_contribution(a, b, c, o);
	} else if (count == 0) {
		return Vector4(0, 0, 0, 0);
	}

	// One vertex below -> single clipped triangle
	if (count == 1) {
		if (below_a) {
			Vector3 p1 = _intersect(a, b, _a, _b);
			Vector3 p2 = _intersect(a, c, _a, _c);
			_submerged_verts.append(a);
			_submerged_verts.append(p1);
			_submerged_verts.append(p2);
			return _tri_contribution(a, p1, p2, o);
		} else if (below_b) {
			Vector3 p1 = _intersect(b, c, _b, _c);
			Vector3 p2 = _intersect(b, a, _b, _a);
			_submerged_verts.append(b);
			_submerged_verts.append(p1);
			_submerged_verts.append(p2);
			return _tri_contribution(b, p1, p2, o);
		} else {
			Vector3 p1 = _intersect(c, a, _c, _a);
			Vector3 p2 = _intersect(c, b, _c, _b);
			_submerged_verts.append(c);
			_submerged_verts.append(p1);
			_submerged_verts.append(p2);
			return _tri_contribution(c, p1, p2, o);
		}
	}

	// Two vertices below -> quad (two triangles)
	else {
		if (!below_a) {
			Vector3 p1 = _intersect(b, a, _b, _a);
			Vector3 p2 = _intersect(c, a, _c, _a);
			_submerged_verts.append(b);
			_submerged_verts.append(c);
			_submerged_verts.append(p2);
			_submerged_verts.append(b);
			_submerged_verts.append(p2);
			_submerged_verts.append(p1);
			Vector4 tri1 = _tri_contribution(b, c, p2, o);
			Vector4 tri2 = _tri_contribution(b, p2, p1, o);
			C += Vector3(tri1.x, tri1.y, tri1.z);
			V += tri1.w;
			C += Vector3(tri2.x, tri2.y, tri2.z);
			V += tri2.w;
		} else if (!below_b) {
			Vector3 p1 = _intersect(c, b, _c, _b);
			Vector3 p2 = _intersect(a, b, _a, _b);
			_submerged_verts.append(c);
			_submerged_verts.append(a);
			_submerged_verts.append(p2);
			_submerged_verts.append(c);
			_submerged_verts.append(p2);
			_submerged_verts.append(p1);
			Vector4 tri1 = _tri_contribution(c, a, p2, o);
			Vector4 tri2 = _tri_contribution(c, p2, p1, o);
			C += Vector3(tri1.x, tri1.y, tri1.z);
			V += tri1.w;
			C += Vector3(tri2.x, tri2.y, tri2.z);
			V += tri2.w;
		} else {
			Vector3 p1 = _intersect(a, c, _a, _c);
			Vector3 p2 = _intersect(b, c, _b, _c);
			_submerged_verts.append(a);
			_submerged_verts.append(b);
			_submerged_verts.append(p2);
			_submerged_verts.append(a);
			_submerged_verts.append(p2);
			_submerged_verts.append(p1);
			Vector4 tri1 = _tri_contribution(a, b, p2, o);
			Vector4 tri2 = _tri_contribution(a, p2, p1, o);
			C += Vector3(tri1.x, tri1.y, tri1.z);
			V += tri1.w;
			C += Vector3(tri2.x, tri2.y, tri2.z);
			V += tri2.w;
		}

		return Vector4(C.x, C.y, C.z, V);
	}
}

#pragma endregion

#pragma region Core Calculations

void MeshBuoyancy::update_statics(const Transform3D &collider_transform) {
	if (!_buoyancy_mesh.is_valid() || _buoyancy_mesh->get_surface_count() == 0) {
		return;
	}

	_vertex = _buoyancy_mesh->get_faces();

	// Calculate volume and centroid
	_mesh_volume = 0.0f;
	_mesh_centroid = Vector3(0, 0, 0);
	Vector3 o = Vector3(0, 0, 0);

	// Integrate tetrahedrons formed by each triangle and the origin
	int face_count = _vertex.size() / 3;
	for (int idx = 0; idx < face_count; ++idx) {
		Vector3 a = _vertex[idx * 3 + 0];
		Vector3 b = _vertex[idx * 3 + 1];
		Vector3 c = _vertex[idx * 3 + 2];

		Vector4 tri = _tri_contribution(a, b, c, o);
		_mesh_centroid += Vector3(tri.x, tri.y, tri.z);
		_mesh_volume += tri.w;
	}

	if (!Math::is_zero_approx(_mesh_volume)) {
		_mesh_centroid /= (_mesh_volume * 4.0f / 3.0f);
		_mesh_centroid = collider_transform.xform(_mesh_centroid);
	} else {
		_mesh_centroid = collider_transform.origin;
	}

	// Auto detect negative volume and recalculate with sign flipped
	if (_mesh_volume < 0.0f) {
		_sign = -_sign;
		update_statics(collider_transform);
	}
}

void MeshBuoyancy::update_dynamics(const Transform3D &collider_global_transform, const Transform3D &collider_local_transform) {
	_submerged_verts.clear();

	// Prepare depth cache
	_depth_map.clear();
	_depth_map.reserve(_vertex.size());
	_depths.resize(_vertex.size());

	_buoyancy_normal = Vector3(0, 0, 0);

	// Dedupe the vertices and calculate depths.
	// For each unique vertex, calculate its depth once and cache the transform.
	// This avoids multiple expensive calls out to LiquidArea per unique vertex.
	for (int idx = 0; idx < _vertex.size(); ++idx) {
		Vector3 global_vertex = collider_global_transform.xform(_vertex[idx]);
		Transform3D xform; // Identity

		// Cache hit, reuse
		if (_depth_map.has(global_vertex)) {
			xform = _depth_map[global_vertex];
		} else {
			// Cache miss, calculate
			if (_liquid_area) {
				if (_ignore_waves) {
					float liquid_y = _liquid_area->get_global_transform().origin.y;
					xform = Transform3D(Basis(), Vector3(global_vertex.x, liquid_y, global_vertex.z));
				} else {
					xform = _liquid_area->get_liquid_transform(global_vertex);
				}
			}
			// Update cache
			_depth_map[global_vertex] = xform;
		}

		// Set depth for this vertex
		_depths[idx] = global_vertex.y - xform.origin.y;

		// Average in this normal
		_buoyancy_normal += xform.basis.get_column(1) / (float)_vertex.size();
	}

	_buoyancy_normal = _buoyancy_normal.normalized();

	// Accumulate submerged volume for each face
	int face_count = _vertex.size() / 3;

	Vector3 o = Vector3(0, 0, 0);
	_submerged_centroid = Vector3(0, 0, 0);
	_submerged_volume = 0.0f;

	for (int idx = 0; idx < face_count; ++idx) {
		// Volume calculation in local space
		Vector3 a = _vertex[idx * 3 + 0];
		Vector3 b = _vertex[idx * 3 + 1];
		Vector3 c = _vertex[idx * 3 + 2];

		float _a = _depths[idx * 3 + 0];
		float _b = _depths[idx * 3 + 1];
		float _c = _depths[idx * 3 + 2];

		// Calculate submerged contribution
		Vector4 submerged_tri = _partial_intersection(a, b, c, o, _a, _b, _c, true);
		_submerged_centroid += Vector3(submerged_tri.x, submerged_tri.y, submerged_tri.z);
		_submerged_volume += submerged_tri.w;
	}

	// Transform submerged centroid from local to body space
	if (!Math::is_zero_approx(_submerged_volume)) {
		_submerged_centroid /= (_submerged_volume * 4.0f / 3.0f);
		_submerged_centroid = collider_local_transform.xform(_submerged_centroid);
	} else {
		_submerged_centroid = collider_local_transform.origin;
	}
}

void MeshBuoyancy::update_forces(const Transform3D &body_transform, const Vector3 &gravity) {
	if (_liquid_area == nullptr || Math::is_zero_approx(_mesh_volume)) {
		_buoyancy_force = Vector3(0, 0, 0);
		_force_position = body_transform.origin;
		return;
	}

	if (_submerged_volume > 0.0f) {
		float liquid_density = _liquid_area->get_density();
		float ratio = _submerged_volume / _mesh_volume;

		// Blend wave normal toward vertical based on submersion ratio
		Vector3 wave_normal = _buoyancy_normal.lerp(Vector3(0, 1, 0), ratio);

		// Scale force so that _buoyancy=1 is neutral, _buoyancy=0 sinks
		float buoyancy_scalar = 1.0f;
		if (_buoyancy != INFINITY && _mass > 0.0f && !Math::is_zero_approx(_mesh_volume)) {
			buoyancy_scalar = _buoyancy * _mass / (liquid_density * _mesh_volume);
		}

		// F_B = buoyancy * rho * V_submerged * g
		_buoyancy_force = wave_normal * buoyancy_scalar * liquid_density * _submerged_volume * -gravity.y;

		// Add current force scaled by submerged volume
		Vector3 current_force = _liquid_area->get_current_speed() * liquid_density * _submerged_volume;
		_buoyancy_force += current_force;

		// Apply force at the submerged centroid in global space
		_force_position = body_transform.xform(_submerged_centroid);
	} else {
		_buoyancy_force = Vector3(0, 0, 0);
		_force_position = body_transform.origin;
	}
}

#pragma endregion
