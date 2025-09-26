#pragma once

#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/classes/wrapped.hpp>
#include <godot_cpp/templates/local_vector.hpp>
#include <godot_cpp/templates/pair.hpp>
#include <godot_cpp/variant/variant.hpp>

#include "property_utils.h"

#define MAX_ROPE_SIDES 64

using namespace godot;

class Rope;
class RopeAttachmentsBase;

class RopeAppearance : public Resource {
	friend Rope;
	GDCLASS(RopeAppearance, Resource)

	float _rope_width = 0.25;
	int _rope_sides = -1;
	float _rope_twist = 1.0;
	int _rope_lod = 2;
	Ref<Material> _material = nullptr;
	Ref<ArrayMesh> _mesh = nullptr;

	NodePath _start_attachment;
	float _start_offset = 0.0;

	Ref<RopeAttachmentsBase> _attachments;

	NodePath _end_attachment;
	float _end_offset = 0.0;

	float _particles_per_meter = 2;

protected:
	static void _bind_methods();
	void _notify_change() {
		emit_changed();
	}

public:
	RopeAppearance() = default;
	virtual ~RopeAppearance() override = default;

	// Exported Properties
	PROPERTY_GET_SET(float, rope_width, _notify_change())

	void set_rope_sides(int p_rope_sides);
	int get_rope_sides() const;

	PROPERTY_GET_SET(float, rope_twist, _notify_change())
	PROPERTY_GET_SET(int, rope_lod, _notify_change())

	void set_material(const Ref<Material> &p_material);
	Ref<Material> get_material() const;

	void set_array_mesh(const Ref<ArrayMesh> &p_mesh);
	Ref<ArrayMesh> get_array_mesh() const;

	PROPERTY_GET_SET(NodePath, start_attachment, _notify_change())
	PROPERTY_GET_SET(float, start_offset, _notify_change())
	PROPERTY_GET_SET(NodePath, end_attachment, _notify_change())
	PROPERTY_GET_SET(float, end_offset, _notify_change())

	void set_attachments(const Ref<RopeAttachmentsBase> &p_attachments);
	Ref<RopeAttachmentsBase> get_attachments() const;

	// simulation parameters
	PROPERTY_GET_SET(float, particles_per_meter, _notify_change())
};