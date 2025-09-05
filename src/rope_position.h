#pragma once

#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/classes/wrapped.hpp>
#include <godot_cpp/templates/local_vector.hpp>
#include <godot_cpp/templates/pair.hpp>
#include <godot_cpp/variant/variant.hpp>

using namespace godot;

class FastRope;
class RopePosition : public Resource {
	friend FastRope;
	GDCLASS(RopePosition, Resource)

	struct Position {
		Position() = default;
		~Position() = default;

		float position = 0.0;
		NodePath node = "";
	};

	Vector<Position> positions;

protected:
	static void _bind_methods();

	void RopePosition::_get_property_list(List<PropertyInfo> *p_list) const;
	bool RopePosition::_property_can_revert(const StringName &p_name) const;
	bool RopePosition::_property_get_revert(const StringName &p_name, Variant &r_property) const;
	bool RopePosition::_set(const StringName &p_name, const Variant &p_property);
	bool RopePosition::_get(const StringName &p_name, Variant &r_property) const;
	Pair<uint64_t, String> RopePosition::_decode_dynamic_propname(const StringName &p_name) const;

public:
	RopePosition() = default;
	~RopePosition() override = default;

	void set_position_count(int count) {
		if (positions.size() != count) {
			positions.resize(count);
			notify_property_list_changed();
		}
	}
	int get_position_count() {
		return positions.size();
	}

	void set_position(int idx, float val) {
		if (idx >= 0 && idx < positions.size()) {
			Position p = positions.get(idx);
			p.position = val;
			positions.set(idx, p);
		}
	}
	float get_position(int idx) const {
		return positions.get(idx).position;
	}

	void set_node(int idx, const NodePath &val) {
		if (idx >= 0 && idx < positions.size()) {
			Position p = positions.get(idx);
			p.node = val;
			positions.set(idx, p);
		}
	}

	const NodePath &get_node(int idx) const {
		return positions.get(idx).node;
	}
};
