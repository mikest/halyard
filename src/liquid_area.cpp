#include "liquid_area.h"

#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>

using namespace godot;

void LiquidArea::_bind_methods() {
	EXPORT_PROPERTY_RANGED(Variant::FLOAT, density, LiquidArea, "1,10000,1,hide_slider,suffix:kg/m^3");
	EXPORT_PROPERTY(Variant::VECTOR3, current_speed, LiquidArea);

	ClassDB::bind_method(D_METHOD("is_point_submerged", "global_point"), &LiquidArea::is_point_submerged);
	ClassDB::bind_method(D_METHOD("get_liquid_transform", "global_point"), &LiquidArea::get_liquid_transform);

	// Debug properties
	ClassDB::bind_method(D_METHOD("set_show_debug", "show"), &LiquidArea::set_show_debug);
	ClassDB::bind_method(D_METHOD("get_show_debug"), &LiquidArea::get_show_debug);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "show_debug"), "set_show_debug", "get_show_debug");

	ClassDB::bind_method(D_METHOD("set_debug_color", "color"), &LiquidArea::set_debug_color);
	ClassDB::bind_method(D_METHOD("get_debug_color"), &LiquidArea::get_debug_color);
	ADD_PROPERTY(PropertyInfo(Variant::COLOR, "debug_color"), "set_debug_color", "get_debug_color");

	ClassDB::bind_method(D_METHOD("_clear_sampled_transforms"), &LiquidArea::_clear_sampled_transforms);

	// virtuals
	GDVIRTUAL_BIND(update_transforms_for_points, "global_points", "transforms")
}

LiquidArea::LiquidArea() :
		NodeDebug(Object::cast_to<Node>(this)) {
	_set_debug_owner_node(this);

	// sea green
	_debug_color = Color(0.0f, 1.0f, 0.5f, 0.8f);
}

LiquidArea::~LiquidArea() {
}

void LiquidArea::_notification(int p_what) {
	NodeDebug::_debug_notification(p_what);

	switch (p_what) {
		case NOTIFICATION_READY: {
			set_process_internal(true);
			set_physics_process_internal(true);
		} break;

		case NOTIFICATION_EXIT_TREE: {
		} break;

		case NOTIFICATION_INTERNAL_PROCESS: {
		} break;

		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
			// if we clear the points here nodes which are processed earlier won't
			// have their sampled points available for this frame
			if (_show_debug)
				call_deferred("_clear_sampled_transforms");
		} break;
	}
}

void LiquidArea::_clear_sampled_transforms() {
	if (is_debug_mesh_dirty()) {
		_internal_update_debug_mesh();
		set_debug_mesh_dirty(false);
	}

	// mesh is in sync, clear sampled points
	_sampled_transforms.clear();
}

LiquidArea *LiquidArea::get_liquid_area(SceneTree *p_tree) {
	LiquidArea *liquid_area = nullptr;

	if (p_tree) {
		Node *root = p_tree->get_current_scene();
		if (root) {
			auto nodes = root->find_children("*", "LiquidArea", true, false);
			if (nodes.size()) {
				liquid_area = Object::cast_to<LiquidArea>(nodes.front());
			}
		}
	}

	return liquid_area;
}

bool LiquidArea::is_point_submerged(const Vector3 &global_point) const {
	Transform3D liquid_transform = get_global_transform();
	Vector3 local_point = liquid_transform.affine_inverse().xform(global_point);
	return local_point.y < 0.0f;
}

Transform3D LiquidArea::get_liquid_transform(const Vector3 &global_point) const {
	if (GDVIRTUAL_IS_OVERRIDDEN(update_transforms_for_points)) {
		TypedArray<Transform3D> ret_val;
		PackedVector3Array points;
		points.append(global_point);

		GDVIRTUAL_CALL(update_transforms_for_points, points, ret_val);

		// accumulate sampled transform
		if (_show_debug) {
			const_cast<LiquidArea *>(this)->_sampled_transforms.append(ret_val[0]);
			const_cast<LiquidArea *>(this)->set_debug_mesh_dirty();
		}

		return ret_val[0];
	} else {
		// Fast return if not overridden: just return the area's global transform.
		return get_global_transform();
	}
}

void LiquidArea::update_transforms_for_points(const PackedVector3Array &global_points,
		TypedArray<Transform3D> r_transforms) const {
	if (GDVIRTUAL_IS_OVERRIDDEN(update_transforms_for_points)) {
		Transform3D ret_val;
		GDVIRTUAL_CALL(update_transforms_for_points, global_points, r_transforms);

		// accumulate all sampled transforms
		if (_show_debug) {
			int point_count = global_points.size();
			for (int i = 0; i < point_count; i++) {
				Transform3D area_transform = r_transforms[i];
				const_cast<LiquidArea *>(this)->_sampled_transforms.append(area_transform);
			}
			const_cast<LiquidArea *>(this)->set_debug_mesh_dirty();
		}
	} else {
		// Default implementation: fill all transforms with the area's global transform.
		int point_count = global_points.size();
		r_transforms.resize(point_count);
		Transform3D area_transform = get_global_transform();
		for (int i = 0; i < point_count; i++) {
			r_transforms[i] = area_transform;
		}

		// accumulate one, as they are all the same
		if (_show_debug) {
			const_cast<LiquidArea *>(this)->_sampled_transforms.append(area_transform);
			const_cast<LiquidArea *>(this)->set_debug_mesh_dirty();
		}
	}
}

#pragma region Debug Mesh
void LiquidArea::set_show_debug(bool p_show) {
	if (p_show != _show_debug) {
		_show_debug = p_show;
		set_debug_mesh_dirty(true);
	}
}

bool LiquidArea::get_show_debug() const {
	return _show_debug;
}

void LiquidArea::set_debug_color(const Color &p_color) {
	_debug_color = p_color;
	set_debug_mesh_dirty(true);
}

Color LiquidArea::get_debug_color() const {
	return _debug_color;
}

void LiquidArea::_destroy_debug_mesh() {
	_sampled_transforms.clear();
}

void LiquidArea::_update_debug_mesh() {
	PackedVector3Array vertices;
	PackedInt32Array indices;

	// no sampled points, skip
	int point_count = _sampled_transforms.size();
	if (point_count == 0)
		return;

	Array arrays;
	arrays.resize(Mesh::ARRAY_MAX);

	float size = 0.1f; // crosshair size

	// Draw 3D cross-hairs at each sampled point
	for (int idx = 0; idx < point_count; ++idx) {
		// convert to local space
		Vector3 point = to_local(_sampled_transforms[idx].origin);

		// X axis line
		vertices.append(point + Vector3(-size, 0, 0));
		vertices.append(point + Vector3(size, 0, 0));

		// Y axis line
		vertices.append(point + Vector3(0, -size, 0));
		vertices.append(point + Vector3(0, size, 0));

		// Z axis line
		vertices.append(point + Vector3(0, 0, -size));
		vertices.append(point + Vector3(0, 0, size));

		// Indices for the 3 lines (6 vertices per point)
		int base = idx * 6;
		indices.append(base + 0);
		indices.append(base + 1);

		indices.append(base + 2);
		indices.append(base + 3);

		indices.append(base + 4);
		indices.append(base + 5);
	}

	// add a normal line for each point that points to Y-up
	for (int idx = 0; idx < point_count; ++idx) {
		// convert to local space
		Transform3D liquid_xform = get_global_transform().affine_inverse() * _sampled_transforms[idx];
		Vector3 origin = liquid_xform.origin;

		Vector3 basis_y = liquid_xform.basis.get_column(1);
		Vector3 point = origin + basis_y * 0.1f; // offset a bit above the surface
		Vector3 normal_end = point + basis_y * size * 2.0f;
		vertices.append(point);
		vertices.append(normal_end);
		int base = point_count * 6 + idx * 2;
		indices.append(base + 0);
		indices.append(base + 1);
	}

	arrays[Mesh::ARRAY_VERTEX] = vertices;
	arrays[Mesh::ARRAY_INDEX] = indices;

	int surf_lines = _debug_mesh->get_surface_count();
	_debug_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_LINES, arrays);

	_debug_material->set_albedo(_debug_color);
	_debug_mesh->surface_set_material(surf_lines, _debug_material);

	// update transform to align with ourself
	_debug_mesh_instance->set_global_transform(get_global_transform());
}

#pragma endregion
