/*
 * Copyright (c) 2026 M. Estee.
 * Licensed under the MIT License.
 */

#pragma once

#include <godot_cpp/classes/curve3d.hpp>
#include <godot_cpp/classes/geometry_instance3d.hpp>
#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/templates/local_vector.hpp>
#include <godot_cpp/variant/basis.hpp>
#include <godot_cpp/variant/transform3d.hpp>

#include "rope_mesh.h"

using namespace godot;

class Pipe : public GeometryInstance3D {
	GDCLASS(Pipe, GeometryInstance3D)

private:
	Ref<Curve3D> _curve;
	float _elbow_radius = 1.0f;
	float _elbow_length = 0.2f;
	float _elbow_scale = 1.1f;
	float _flange_length = 0.2f;
	float _flange_scale = 1.2f;
	Ref<Material> _elbow_material;
	Ref<Material> _pipe_material;
	int _sides = 3;
	float _radius = 1.0f;
	float _twist = 1.0f;
	Ref<RopeMesh> _rope_mesh;

	LocalVector<Transform3D> _frames;
	bool _dirty = false;

	void _notification(int p_what);
	void _rebind_curve_changed_signal();
	void _request_rebuild();

	void _build_frames();
	Transform3D _calculate_frame(const Vector3 &p_first, const Vector3 &p_second, float p_tilt, const Basis &p_basis_prev) const;
	Basis _rotate_to_align(Basis p_basis, const Vector3 &p_start, const Vector3 &p_end) const;

	void _on_curve_changed();
	void _update_aabb();
	void _update_mesh();

protected:
	static void _bind_methods();

public:
	Pipe();
	virtual ~Pipe() override = default;

	void set_curve(const Ref<Curve3D> &p_curve);
	Ref<Curve3D> get_curve() const;

	void set_elbow_radius(float p_elbow_radius);
	float get_elbow_radius() const;

	void set_elbow_length(float p_elbow_length);
	float get_elbow_length() const;

	void set_elbow_scale(float p_elbow_scale);
	float get_elbow_scale() const;

	void set_flange_length(float p_flange_length);
	float get_flange_length() const;

	void set_flange_scale(float p_flange_scale);
	float get_flange_scale() const;

	void set_elbow_material(const Ref<Material> &p_elbow_material);
	Ref<Material> get_elbow_material() const;

	void set_pipe_material(const Ref<Material> &p_pipe_material);
	Ref<Material> get_pipe_material() const;

	void set_sides(int p_sides);
	int get_sides() const;

	void set_radius(float p_radius);
	float get_radius() const;

	void set_twist(float p_twist);
	float get_twist() const;
};
