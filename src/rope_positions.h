#pragma once

#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/classes/wrapped.hpp>
#include <godot_cpp/templates/local_vector.hpp>
#include <godot_cpp/templates/pair.hpp>
#include <godot_cpp/variant/variant.hpp>

using namespace godot;

const int MAX_ROPE_POSITIONS = 10;

class FastRope;
class RopePositions : public Resource {
	friend FastRope;
	GDCLASS(RopePositions, Resource)

	struct Position {
		Position() = default;
		~Position() = default;

		float position = 0.0;
		NodePath node = "";
	};

	Vector<Position> _positions;

	// dummy so we can return something without an access exception
	NodePath _ERROR_DUMMY_NODEPATH;

protected:
	static void _bind_methods();

	void RopePositions::_get_property_list(List<PropertyInfo> *p_list) const;
	bool RopePositions::_property_can_revert(const StringName &p_name) const;
	bool RopePositions::_property_get_revert(const StringName &p_name, Variant &r_property) const;
	bool RopePositions::_set(const StringName &p_name, const Variant &p_property);
	bool RopePositions::_get(const StringName &p_name, Variant &r_property) const;
	Pair<uint64_t, String> RopePositions::_decode_dynamic_propname(const StringName &p_name) const;

public:
	RopePositions() = default;
	~RopePositions() override = default;

	void set_position_count(int count) {
		if (count < 0 || count > MAX_ROPE_POSITIONS) {
			print_error("RopePositions: invalid count passed to set_position_count(), max_count:" + itos(MAX_ROPE_POSITIONS));
			return;
		}

		if (_positions.size() != count) {
			_positions.resize(count);
			notify_property_list_changed();
		}
	}
	int get_position_count() {
		return _positions.size();
	}

	void set_position(int idx, float val) {
		if (idx < 0 || idx >= _positions.size()) {
			print_error("RopePositions: invalid index passed to set_position()");
			return;
		}

		Position p = _positions.get(idx);
		p.position = val;
		_positions.set(idx, p);
		notify_property_list_changed();
	}

	float get_position(int idx) const {
		if (idx < 0 || idx >= _positions.size()) {
			print_error("RopePositions: invalid index passed to get_position()");
			return -1.0;
		}

		return _positions.get(idx).position;
	}

	void set_node(int idx, const NodePath &val) {
		if (idx < 0 || idx >= _positions.size()) {
			print_error("RopePositions: invalid index passed to set_node()");
			return;
		}

		Position p = _positions.get(idx);
		p.node = val;
		_positions.set(idx, p);
		notify_property_list_changed();
	}

	const NodePath &get_node(int idx) const {
		if (idx < 0 || idx >= _positions.size()) {
			print_error("RopePositions: invalid index passed to get_node()");
			return _ERROR_DUMMY_NODEPATH;
		}

		return _positions.get(idx).node;
	}
};
