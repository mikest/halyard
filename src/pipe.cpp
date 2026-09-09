/*
 * Copyright (c) 2026 M. Estee.
 * Licensed under the MIT License.
 */

#include "pipe.h"

#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/core/math.hpp>

// Utility structure for calculating elbow geometry in a pipe.
struct ElbowGeometry {
	float elbow_radius = 0.0f;
	Basis basis;
	Vector3 normal;
	Vector3 tangent;
	Vector3 binormal;
	Vector3 center;
	Vector3 tangent_start;
	Vector3 tangent_end;
	float angle = 0.0f;

	// util for avoiding degenerate normal
	static Vector3 stable_reference_normal(const Vector3 &p_direction) {
		if (Math::abs(p_direction.x) < 0.9f) {
			return Vector3(1, 0, 0);
		}
		return Vector3(0, 1, 0);
	}

	// Find the center of a circle transcribing two lines joined at a point in a 2D plane.
	Vector3 calculate_elbow_center(const Vector3 &p_point, const Vector3 &p_incoming, const Vector3 &p_outgoing) const {
		Vector3 bisector = (p_incoming + p_outgoing).normalized();
		Vector3 inward_normal_in = normal.cross(p_incoming).normalized();
		Vector3 inward_normal_out = normal.cross(p_outgoing).normalized();

		if (inward_normal_in.dot(bisector) < 0.0f) {
			inward_normal_in = -inward_normal_in;
		}
		if (inward_normal_out.dot(bisector) > 0.0f) {
			inward_normal_out = -inward_normal_out;
		}

		// The offset line is parallel to the original segment and sits elbow_radius away from it.
		// For any point on the original segment, the perpendicular distance to this line is elbow_radius.
		Vector3 incoming_offset_origin = p_point + inward_normal_in * elbow_radius;
		Vector3 outgoing_offset_origin = p_point + inward_normal_out * elbow_radius;

		Vector2 line_in_origin(
				(incoming_offset_origin - p_point).dot(tangent),
				(incoming_offset_origin - p_point).dot(binormal));
		Vector2 line_out_origin(
				(outgoing_offset_origin - p_point).dot(tangent),
				(outgoing_offset_origin - p_point).dot(binormal));

		Vector2 line_in_dir(p_incoming.dot(tangent), p_incoming.dot(binormal));
		Vector2 line_out_dir(p_outgoing.dot(tangent), p_outgoing.dot(binormal));

		float line_denominator = line_in_dir.cross(line_out_dir);
		if (Math::abs(line_denominator) <= 0.000001f) {
			line_denominator = 1.0f;
		}

		Vector2 center_2d = line_in_origin + line_in_dir * ((line_out_origin - line_in_origin).cross(line_out_dir) / line_denominator);
		return p_point + tangent * center_2d.x + binormal * center_2d.y;
	}

	ElbowGeometry(float p_radius, const Vector3 &p_point, const Vector3 &p_incoming, const Vector3 &p_outgoing) {
		elbow_radius = p_radius;

		// force normalized
		Vector3 incoming = p_incoming.normalized();
		Vector3 outgoing = p_outgoing.normalized();

		// calc normal for the plane
		normal = incoming.cross(outgoing);
		if (normal.length_squared() < 0.000001f) {
			normal = stable_reference_normal(incoming);
			normal = normal - incoming * incoming.dot(normal);
			normal = normal.normalized();
		} else {
			normal = normal.normalized();
		}

		// plane tangent and bitangent
		tangent = incoming.normalized();
		binormal = normal.cross(tangent).normalized();
		basis = Basis(tangent, normal, binormal);

		// ...uses plane_normal, plane_x, plane_y and elbow_radius
		center = calculate_elbow_center(p_point, incoming, outgoing);

		// points on the lines where the curve starts/ends
		tangent_start = p_point + incoming * ((center - p_point).dot(incoming));
		tangent_end = p_point + outgoing * ((center - p_point).dot(outgoing));

		// angle of the sharp
		angle = Math::acos(Math::clamp(incoming.dot(outgoing), -1.0f, 1.0f));
	}
};


void Pipe::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_curve", "curve"), &Pipe::set_curve);
	ClassDB::bind_method(D_METHOD("get_curve"), &Pipe::get_curve);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve3D"), "set_curve", "get_curve");

	ClassDB::bind_method(D_METHOD("set_elbow_radius", "elbow_radius"), &Pipe::set_elbow_radius);
	ClassDB::bind_method(D_METHOD("get_elbow_radius"), &Pipe::get_elbow_radius);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "elbow_radius", PROPERTY_HINT_RANGE, "0,100,0.01,or_greater"), "set_elbow_radius", "get_elbow_radius");

	ClassDB::bind_method(D_METHOD("set_elbow_length", "elbow_length"), &Pipe::set_elbow_length);
	ClassDB::bind_method(D_METHOD("get_elbow_length"), &Pipe::get_elbow_length);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "elbow_length", PROPERTY_HINT_RANGE, "0,100,0.01,or_greater"), "set_elbow_length", "get_elbow_length");

	ClassDB::bind_method(D_METHOD("set_elbow_scale", "elbow_scale"), &Pipe::set_elbow_scale);
	ClassDB::bind_method(D_METHOD("get_elbow_scale"), &Pipe::get_elbow_scale);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "elbow_scale", PROPERTY_HINT_RANGE, "0,100,0.01,or_greater"), "set_elbow_scale", "get_elbow_scale");

	ClassDB::bind_method(D_METHOD("set_flange_length", "flange_length"), &Pipe::set_flange_length);
	ClassDB::bind_method(D_METHOD("get_flange_length"), &Pipe::get_flange_length);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "flange_length", PROPERTY_HINT_RANGE, "0,100,0.01,or_greater"), "set_flange_length", "get_flange_length");

	ClassDB::bind_method(D_METHOD("set_flange_scale", "flange_scale"), &Pipe::set_flange_scale);
	ClassDB::bind_method(D_METHOD("get_flange_scale"), &Pipe::get_flange_scale);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "flange_scale", PROPERTY_HINT_RANGE, "0,100,0.01,or_greater"), "set_flange_scale", "get_flange_scale");

	ClassDB::bind_method(D_METHOD("set_elbow_material", "elbow_material"), &Pipe::set_elbow_material);
	ClassDB::bind_method(D_METHOD("get_elbow_material"), &Pipe::get_elbow_material);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "elbow_material", PROPERTY_HINT_RESOURCE_TYPE, "Material"), "set_elbow_material", "get_elbow_material");

	ClassDB::bind_method(D_METHOD("set_pipe_material", "pipe_material"), &Pipe::set_pipe_material);
	ClassDB::bind_method(D_METHOD("get_pipe_material"), &Pipe::get_pipe_material);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "pipe_material", PROPERTY_HINT_RESOURCE_TYPE, "Material"), "set_pipe_material", "get_pipe_material");

	ClassDB::bind_method(D_METHOD("set_sides", "sides"), &Pipe::set_sides);
	ClassDB::bind_method(D_METHOD("get_sides"), &Pipe::get_sides);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "sides", PROPERTY_HINT_RANGE, "3,128,1,or_greater"), "set_sides", "get_sides");

	ClassDB::bind_method(D_METHOD("set_radius", "radius"), &Pipe::set_radius);
	ClassDB::bind_method(D_METHOD("get_radius"), &Pipe::get_radius);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "radius", PROPERTY_HINT_RANGE, "0.001,10,0.001,or_greater"), "set_radius", "get_radius");

	ClassDB::bind_method(D_METHOD("set_twist", "twist"), &Pipe::set_twist);
	ClassDB::bind_method(D_METHOD("get_twist"), &Pipe::get_twist);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "twist", PROPERTY_HINT_RANGE, "0,10,0.01,or_greater"), "set_twist", "get_twist");

	ClassDB::bind_method(D_METHOD("_on_curve_changed"), &Pipe::_on_curve_changed);
}

Pipe::Pipe() {
	_rope_mesh.instantiate();
	if (_rope_mesh.is_valid()) {
		_rope_mesh->set_sides(_sides);
		_rope_mesh->set_radius(_radius);
		_rope_mesh->set_rope_twist(_twist);
		set_base(_rope_mesh->get_rid());
	}
}

void Pipe::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_READY: {
			if (_rope_mesh.is_valid()) {
				set_base(_rope_mesh->get_rid());
			}

			_rebind_curve_changed_signal();

			if (_rope_mesh.is_valid() && (_rope_mesh->get_surface_count() == 0 || _dirty)) {
				_update_mesh();
			}
		} break;
		case NOTIFICATION_PREDELETE: {
			set_base(RID());
		} break;
		default:
			break;
	}
}

void Pipe::_request_rebuild() {
	_dirty = true;
	if (is_inside_tree()) {
		_update_mesh();
	}
}

void Pipe::_on_curve_changed() {
	_request_rebuild();
}

void Pipe::_rebind_curve_changed_signal() {
	Callable callback(this, "_on_curve_changed");

	if (_curve.is_valid() && _curve->is_connected("changed", callback)) {
		_curve->disconnect("changed", callback);
	}

	if (_curve.is_valid()) {
		_curve->connect("changed", callback);
	}
}

void Pipe::set_curve(const Ref<Curve3D> &p_curve) {
	if (_curve == p_curve) {
		return;
	}

	Callable callback(this, "_on_curve_changed");
	if (_curve.is_valid() && _curve->is_connected("changed", callback)) {
		_curve->disconnect("changed", callback);
	}

	_curve = p_curve;
	_rebind_curve_changed_signal();
	_request_rebuild();
}

Ref<Curve3D> Pipe::get_curve() const {
	return _curve;
}

void Pipe::set_elbow_radius(float p_elbow_radius) {
	if (Math::is_equal_approx(_elbow_radius, p_elbow_radius)) {
		return;
	}
	_elbow_radius = p_elbow_radius;
	_request_rebuild();
}

float Pipe::get_elbow_radius() const {
	return _elbow_radius;
}

void Pipe::set_elbow_length(float p_elbow_length) {
	if (Math::is_equal_approx(_elbow_length, p_elbow_length)) {
		return;
	}
	_elbow_length = p_elbow_length;
	_request_rebuild();
}

float Pipe::get_elbow_length() const {
	return _elbow_length;
}

void Pipe::set_elbow_scale(float p_elbow_scale) {
	if (Math::is_equal_approx(_elbow_scale, p_elbow_scale)) {
		return;
	}
	_elbow_scale = p_elbow_scale;
	_request_rebuild();
}

float Pipe::get_elbow_scale() const {
	return _elbow_scale;
}

void Pipe::set_flange_length(float p_flange_length) {
	if (Math::is_equal_approx(_flange_length, p_flange_length)) {
		return;
	}
	_flange_length = p_flange_length;
	_request_rebuild();
}

float Pipe::get_flange_length() const {
	return _flange_length;
}

void Pipe::set_flange_scale(float p_flange_scale) {
	if (Math::is_equal_approx(_flange_scale, p_flange_scale)) {
		return;
	}
	_flange_scale = p_flange_scale;
	_request_rebuild();
}

float Pipe::get_flange_scale() const {
	return _flange_scale;
}

void Pipe::set_elbow_material(const Ref<Material> &p_elbow_material) {
	if (_elbow_material == p_elbow_material) {
		return;
	}
	_elbow_material = p_elbow_material;
	_request_rebuild();
}

Ref<Material> Pipe::get_elbow_material() const {
	return _elbow_material;
}

void Pipe::set_pipe_material(const Ref<Material> &p_pipe_material) {
	if (_pipe_material == p_pipe_material) {
		return;
	}
	_pipe_material = p_pipe_material;
	_request_rebuild();
}

Ref<Material> Pipe::get_pipe_material() const {
	return _pipe_material;
}

void Pipe::set_sides(int p_sides) {
	int sides = Math::max(p_sides, 3);
	if (_sides == sides) {
		return;
	}
	_sides = sides;
	if (_rope_mesh.is_valid()) {
		_rope_mesh->set_sides(_sides);
	}
	_request_rebuild();
}


int Pipe::get_sides() const {
	return _sides;
}

void Pipe::set_radius(float p_radius) {
	float radius = Math::max(p_radius, 0.001f);
	if (Math::is_equal_approx(_radius, radius)) {
		return;
	}
	_radius = radius;
	if (_rope_mesh.is_valid()) {
		_rope_mesh->set_radius(_radius);
	}
	_request_rebuild();
}

float Pipe::get_radius() const {
	return _radius;
}

void Pipe::set_twist(float p_twist) {
	if (Math::is_equal_approx(_twist, p_twist)) {
		return;
	}
	_twist = p_twist;
	if (_rope_mesh.is_valid()) {
		_rope_mesh->set_rope_twist(_twist);
	}
	_request_rebuild();
}

float Pipe::get_twist() const {
	return _twist;
}

void Pipe::_update_aabb() {
	AABB aabb;
	if (_rope_mesh.is_valid()) {
		aabb = _rope_mesh->get_custom_aabb();
	}
	set_custom_aabb(aabb);
}

void Pipe::_update_mesh() {
	if (!_rope_mesh.is_valid()) {
		return;
	}

	if (!_curve.is_valid()) {
		_rope_mesh->clear_mesh();
		_update_aabb();
		_dirty = false;
		return;
	}

	_rope_mesh->clear_mesh();
	_rope_mesh->set_rope_length(_curve->get_baked_length());
	_build_frames();

	// pipe
	if (_frames.size() >= 2) {
		_rope_mesh->begin_update_mesh();

		Transform3D first = _frames[0];
		_rope_mesh->emit_endcap(true, first);

		// convert the frames to orthonormalized versions
		LocalVector<Transform3D> tube;
		tube.reserve(_frames.size());
		for (int idx = 0; idx < _frames.size(); idx++) {
			tube.push_back(_frames[idx].orthonormalized());
		}

		// emit the tube
		_rope_mesh->emit_tube(tube);

		Transform3D last = _frames[_frames.size() - 1];
		_rope_mesh->emit_endcap(false, last);
		_rope_mesh->end_update_mesh(Ref<Material>());
	}

	if (_rope_mesh->get_surface_count() > 0) {
		_rope_mesh->surface_set_material(0, _pipe_material);
	}

	// elbows
	if (_frames.size() >= 2) {
		// emit sections for frames
		float length = 0.0f;
		LocalVector<Transform3D> elbow;

		for (int idx = 1; idx < _frames.size(); idx++) {
			Transform3D prev = _frames[idx - 1];
			Transform3D curr = _frames[idx];

			bool prev_is_elbow = !prev.basis.is_orthonormal();
			bool curr_is_elbow = !curr.basis.is_orthonormal();

			// construct elbow begin
			if (!prev_is_elbow && curr_is_elbow) {
				_rope_mesh->begin_update_mesh();
				elbow.clear();
				length = 0.0f;
				elbow.push_back(prev);
				elbow.push_back(curr);
			// add segements
			} else if (prev_is_elbow && curr_is_elbow) {
				elbow.push_back(curr);
				length += prev.origin.distance_to(curr.origin);
			// elbow end
			} else if (prev_is_elbow && !curr_is_elbow) {
				_rope_mesh->set_rope_length(length);
				elbow.push_back(curr);
				_rope_mesh->emit_tube(elbow);
				_rope_mesh->end_update_mesh(Ref<Material>());
				_rope_mesh->surface_set_material(_rope_mesh->get_surface_count() - 1, _elbow_material);
			}
		}
	}

	_update_aabb();
	_dirty = false;
}

void Pipe::_build_frames() {
	_frames.clear();

	// Need at least two points to calculate
	if (!_curve.is_valid() || _curve->get_point_count() < 2 || !_rope_mesh.is_valid()) {
		return;
	}

	// X right, Y up, -Z forward
	Basis basis_prev;
	Transform3D frame_prev;
	float tilt_prev = 0.0f;

	// Initial frame
	const float unit_epsilon = 0.001f;
	Vector3 first = _curve->get_point_position(0);
	Vector3 second = _curve->get_point_position(1);
	Vector3 forward = first.direction_to(second);

	if (Math::abs(forward.dot(Vector3(0, 1, 0))) > 1.0f - unit_epsilon) {
		basis_prev = Basis::looking_at(forward, Vector3(1, 0, 0));
	} else {
		basis_prev = Basis::looking_at(forward, Vector3(0, 1, 0));
	}

	Transform3D frame(basis_prev, first);
	frame = frame.rotated_local(Vector3(0, 0, 1), _curve->get_point_tilt(0));
	_frames.push_back(frame);
	frame_prev = frame;
	tilt_prev = _curve->get_point_tilt(0);

	// Parallel transport through middle section
	for (int idx = 1; idx < _curve->get_point_count() - 1; idx++) {
		Vector3 prev = _curve->get_point_position(idx - 1);
		Vector3 curr = _curve->get_point_position(idx);
		Vector3 next = _curve->get_point_position(idx + 1);

		Vector3 incoming = prev.direction_to(curr);
		Vector3 outgoing = curr.direction_to(next);

		ElbowGeometry geo(_elbow_radius, curr, incoming, outgoing);

		Vector3 flange_start = geo.tangent_start - incoming * (_elbow_length + _flange_length);
		Vector3 elbow_start = geo.tangent_start - incoming * _elbow_length;
		Vector3 elbow_end = geo.tangent_end + outgoing * _elbow_length;
		Vector3 flange_end = geo.tangent_end + outgoing * (_elbow_length + _flange_length);

		// prev to elbow start
		float tilt = Math::lerp(tilt_prev, _curve->get_point_tilt(0), 0.5f);
		frame = _calculate_frame(_curve->get_point_position(idx - 1), flange_start, tilt, basis_prev);
		_frames.push_back(frame.scaled_local(Vector3(1.0f, 1.0f, 1.0f)));
		_frames.push_back(frame.scaled_local(Vector3(_flange_scale, _flange_scale, _flange_scale)));
		basis_prev = frame.basis;
		frame_prev = frame;
		tilt_prev = tilt;

		// elbow sleeve
		frame = _calculate_frame(flange_start, elbow_start, tilt, basis_prev);
		_frames.push_back(frame.scaled_local(Vector3(_flange_scale, _flange_scale, _flange_scale)));
		_frames.push_back(frame.scaled_local(Vector3(_elbow_scale, _elbow_scale, _elbow_scale)));
		basis_prev = frame.basis;
		frame_prev = frame;

		// curved section
		Vector3 start = geo.tangent_start - geo.center;
		int count = int(geo.angle / Math_PI * 2.0f * _rope_mesh->get_sides());
		for (int rdx = 0; rdx < count; rdx++) {
			Transform3D xform;
			xform = xform.translated(start);
			xform = xform.rotated(geo.normal, geo.angle / count * rdx + geo.angle / count * 0.5f);
			xform = xform.translated(geo.center);

			frame = _calculate_frame(frame_prev.origin, xform.origin, tilt, basis_prev);
			_frames.push_back(frame.scaled_local(Vector3(_elbow_scale, _elbow_scale, _elbow_scale)));
			basis_prev = frame.basis;
			frame_prev = frame;
		}

		// elbow sleeve
		frame = _calculate_frame(geo.tangent_end, elbow_end, tilt, basis_prev);
		_frames.push_back(frame.scaled_local(Vector3(_elbow_scale, _elbow_scale, _elbow_scale)));
		_frames.push_back(frame.scaled_local(Vector3(_flange_scale, _flange_scale, _flange_scale)));
		basis_prev = frame.basis;
		frame_prev = frame;

		// elbow end
		frame = _calculate_frame(elbow_end, flange_end, _curve->get_point_tilt(idx), basis_prev);
		_frames.push_back(frame.scaled_local(Vector3(_flange_scale, _flange_scale, _flange_scale)));
		_frames.push_back(frame.scaled_local(Vector3(1.0f, 1.0f, 1.0f)));
		basis_prev = frame.basis;
		frame_prev = frame;
		tilt_prev = _curve->get_point_tilt(idx);
	}

	// Last segment in frames
	int last = _curve->get_point_count() - 1;
	frame = _calculate_frame(_frames[_frames.size() - 1].origin, _curve->get_point_position(last), _curve->get_point_tilt(last), basis_prev);
	_frames.push_back(frame);

	// now rotate the whole frameset for RopeMesh
	for (int idx = 0; idx < _frames.size(); idx++) {
		_frames[idx] = _frames[idx].rotated_local(Vector3(1, 0, 0), Math_PI / 2.0f);
	}
}

Transform3D Pipe::_calculate_frame(const Vector3 &p_first, const Vector3 &p_second, float p_tilt, const Basis &p_basis_prev) const {
	Vector3 forward = p_first.direction_to(p_second);

	Basis rotate = _rotate_to_align(Basis(), -p_basis_prev.get_column(2), forward);
	Basis basis_cur = (rotate * p_basis_prev).orthonormalized();

	Transform3D frame(basis_cur, p_second);
	frame = frame.rotated_local(Vector3(0, 0, 1), p_tilt);
	return frame;
}

Basis Pipe::_rotate_to_align(Basis p_basis, const Vector3 &p_start, const Vector3 &p_end) const {
	// From godot:basis.cpp -> Basis::rotate_to_align
	Vector3 axis = p_start.cross(p_end).normalized();
	if (axis.length_squared() != 0.0f) {
		float dot = p_start.dot(p_end);
		dot = Math::clamp(dot, -1.0f, 1.0f);
		float angle = Math::acos(dot);
		p_basis = Basis(axis, angle) * p_basis;
	}

	return p_basis;
}
