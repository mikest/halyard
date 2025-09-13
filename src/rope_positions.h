#pragma once

#include "property_utils.h"
#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/classes/wrapped.hpp>
#include <godot_cpp/core/binder_common.hpp>
#include <godot_cpp/core/gdvirtual.gen.inc>
#include <godot_cpp/templates/local_vector.hpp>
#include <godot_cpp/templates/pair.hpp>
#include <godot_cpp/variant/variant.hpp>

using namespace godot;

const int MAX_ROPE_POSITIONS = 30;

class Rope;
class RopePositions : public Resource {
	friend Rope;
	GDCLASS(RopePositions, Resource)

	struct Position {
		Position() = default;
		~Position() = default;

		float position = 0.0;
		NodePath node = "";
	};

	Vector<Position> _positions;

protected:
	static void _bind_methods();

	void _get_property_list(List<PropertyInfo> *p_list) const;
	bool _property_can_revert(const StringName &p_name) const;
	bool _property_get_revert(const StringName &p_name, Variant &r_property) const;
	bool _set(const StringName &p_name, const Variant &p_property);
	bool _get(const StringName &p_name, Variant &r_property) const;

	Pair<uint64_t, String> _get_propname_with_index(const StringName &p_name) const;

public:
	RopePositions() = default;
	~RopePositions() override = default;

	GDVIRTUAL2RC(float, get_position_at_index, uint64_t, float);
	virtual float get_position_at_index(uint64_t idx, float) const;

	void set_position_count(uint64_t count);
	uint64_t get_position_count() const;

	void set_position(uint64_t idx, float val);
	float get_position(uint64_t idx) const;

	void set_node(uint64_t idx, const NodePath &val);
	const NodePath &get_node(uint64_t idx) const;
};
