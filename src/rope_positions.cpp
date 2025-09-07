#include "rope_positions.h"

void RopePositions::_bind_methods() {
	EXPORT_PROPERTY(Variant::INT, position_count, RopePositions);

	ClassDB::bind_method(D_METHOD("set_position", "index", "position"), &RopePositions::set_position);
	ClassDB::bind_method(D_METHOD("get_position", "index"), &RopePositions::get_position);

	ClassDB::bind_method(D_METHOD("set_node", "index", "node"), &RopePositions::set_node);
	ClassDB::bind_method(D_METHOD("get_node", "index"), &RopePositions::get_node);
}

#define POSITION_PREFIX "position_"

// Build up a dynamic list of properties for each element in the position vector.
void RopePositions::_get_property_list(List<PropertyInfo> *p_list) const {
	for (uint64_t idx = 0; idx < _positions.size(); ++idx) {
		uint32_t usage = PROPERTY_USAGE_DEFAULT;
		String name = POSITION_PREFIX + itos(idx);

		p_list->push_back(PropertyInfo(Variant::NIL, name.capitalize(), PROPERTY_HINT_NONE, name + "/", PROPERTY_USAGE_GROUP));
		p_list->push_back(PropertyInfo(Variant::FLOAT, name + "/position", PROPERTY_HINT_RANGE, "0.0,1.0,0.01", usage));
		p_list->push_back(PropertyInfo(Variant::NODE_PATH, name + "/node", PROPERTY_HINT_NODE_PATH_TO_EDITED_NODE, "", usage));
	}
}

bool RopePositions::_property_can_revert(const StringName &p_name) const {
	if (p_name.begins_with(POSITION_PREFIX)) {
		return true;
	}

	return false;
}

bool RopePositions::_property_get_revert(const StringName &p_name, Variant &r_property) const {
	if (p_name.begins_with(POSITION_PREFIX)) {
		Pair<uint64_t, String> subprop = _get_propname_with_index(p_name);
		String sub_name = subprop.second;
		if (sub_name == "position") {
			r_property = 0.0;
		} else if (sub_name == "node") {
			r_property = "";
		} else {
			return false;
		}
		return true;
	}
	return false;
}

bool RopePositions::_set(const StringName &p_name, const Variant &p_property) {
	if (p_name.begins_with(POSITION_PREFIX)) {
		Pair<uint64_t, String> subprop = _get_propname_with_index(p_name);
		uint64_t idx = subprop.first;
		String sub_name = subprop.second;
		if (sub_name == "position") {
			set_position(idx, p_property);
		} else if (sub_name == "node") {
			set_node(idx, p_property);
		} else {
			return false;
		}
		return true;
	}
	return false;
}

bool RopePositions::_get(const StringName &p_name, Variant &r_property) const {
	if (p_name.begins_with(POSITION_PREFIX)) {
		Pair<uint64_t, String> subprop = _get_propname_with_index(p_name);
		uint64_t idx = subprop.first;
		String sub_name = subprop.second;
		if (sub_name == "position") {
			r_property = get_position(idx);
		} else if (sub_name == "node") {
			r_property = get_node(idx);
		} else {
			return false;
		}
		return true;
	}
	return false;
}

Pair<uint64_t, String> RopePositions::_get_propname_with_index(const StringName &p_name) const {
	PackedStringArray prop_path = p_name.split("/");
	if (prop_path.size() != 2) {
		return Pair<uint64_t, String>();
	}

	uint64_t idx = prop_path[0].trim_prefix(POSITION_PREFIX).to_int();
	String sub_name = prop_path[1];
	return Pair(idx, sub_name);
}

#pragma region Accessors

void RopePositions::set_position_count(uint64_t count) {
	if (count > MAX_ROPE_POSITIONS) {
		print_error("RopePositions: invalid count passed to set_position_count(), max_count:" + itos(MAX_ROPE_POSITIONS));
		return;
	}

	if (_positions.size() != count) {
		_positions.resize(count);
		notify_property_list_changed();
	}
}
uint64_t RopePositions::get_position_count() const {
	return _positions.size();
}

void RopePositions::set_position(uint64_t idx, float val) {
	if (idx >= _positions.size()) {
		print_error("RopePositions: invalid index passed to set_position()");
		return;
	}

	Position p = _positions.get(idx);
	p.position = val;
	_positions.set(idx, p);
}

float RopePositions::get_position(uint64_t idx) const {
	if (idx >= _positions.size()) {
		print_error("RopePositions: invalid index passed to get_position()");
		return -1.0;
	}

	return _positions.get(idx).position;
}

void RopePositions::set_node(uint64_t idx, const NodePath &val) {
	if (idx >= _positions.size()) {
		print_error("RopePositions: invalid index passed to set_node()");
		return;
	}

	Position p = _positions.get(idx);
	p.node = val;
	_positions.set(idx, p);
	notify_property_list_changed();
}

const NodePath &RopePositions::get_node(uint64_t idx) const {
	if (idx >= _positions.size()) {
		print_error("RopePositions: invalid index passed to get_node()");
		// dummy so we can return something without an access exception
		static NodePath _ERROR_DUMMY_NODEPATH = "error";
		return _ERROR_DUMMY_NODEPATH;
	}

	return _positions.get(idx).node;
}

#pragma endregion