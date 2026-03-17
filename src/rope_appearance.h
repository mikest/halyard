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

class RopeAppearance : public Resource {
	friend Rope;
	GDCLASS(RopeAppearance, Resource)

public:
	enum Distribution {
		ABSOLUTE = 0, // Distance is measured from the start or end of the rope.
		RELATIVE,     // Distance is a relative offset from previous attachment.
		UNIFORM,      // Attachments are distributed uniformly along the rope.
		SCALAR,       // Distance is a scalar multiple of the rope length, 0=start 1=end.
	};

private:
	struct Attachment {
		float offset = 0.0;
		bool from_end = false;
		NodePath node_path;
		float _abs_offset = 0.0; // computed, not serialized

		Attachment() = default;
	};

	int _rope_sides = -1;
	float _rope_twist = 1.0;
	int _rope_lod = 2;
	Ref<Material> _material = nullptr;
	Ref<ArrayMesh> _mesh = nullptr;

	float _start_offset = 0.0; // trim rope mesh from the start end
	float _end_offset = 0.0;   // trim rope mesh from the end

	LocalVector<Attachment> _attachments;
	Distribution _attachment_distribution = Distribution::ABSOLUTE;

	void _notify_attachments_changed() {
		notify_property_list_changed();
		emit_changed();
	}

protected:
	static void _bind_methods();

	void _notify_change() {
		emit_changed();
	}

	void _get_property_list(List<PropertyInfo> *p_list) const;
	bool _set(const StringName &p_name, const Variant &p_property);
	bool _get(const StringName &p_name, Variant &r_property) const;

public:
	RopeAppearance() = default;
	virtual ~RopeAppearance() override = default;

	// Exported Properties
	void set_rope_sides(int p_rope_sides);
	int get_rope_sides() const;

	PROPERTY_GET_SET(float, rope_twist, _notify_change())
	PROPERTY_GET_SET(int, rope_lod, _notify_change())

	void set_material(const Ref<Material> &p_material);
	Ref<Material> get_material() const;

	void set_array_mesh(const Ref<ArrayMesh> &p_mesh);
	Ref<ArrayMesh> get_array_mesh() const;

	PROPERTY_GET_SET(float, start_offset, _notify_change())
	PROPERTY_GET_SET(float, end_offset, _notify_change())

	// Attachment list management
	void set_attachment_count(int count);
	int get_attachment_count() const;
	void clear_attachments();

	void set_attachment_offset(int idx, float offset);
	float get_attachment_offset(int idx) const;

	void set_attachment_from(int idx, int from);
	int get_attachment_from(int idx) const;

	void set_attachment_nodepath(int idx, const NodePath &path);
	NodePath get_attachment_nodepath(int idx) const;

	void set_attachment_distribution(int val);
	int get_attachment_distribution() const;
};

VARIANT_ENUM_CAST(RopeAppearance::Distribution);