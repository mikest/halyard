#pragma once
#include "rope_positions.h"

#define POSITION_PREFIX "position_"

// Build up a dynamic list of properties for each element in the position vector.
void RopePositions::_rp_get_property_list(List<PropertyInfo> *p_list) const {
	for (uint64_t idx = 0; idx < _positions.size(); ++idx) {
		uint32_t usage = PROPERTY_USAGE_DEFAULT;
		String name = POSITION_PREFIX + itos(idx);

		p_list->push_back(PropertyInfo(Variant::NIL, name.capitalize(), PROPERTY_HINT_NONE, name + "/", PROPERTY_USAGE_GROUP));
		p_list->push_back(PropertyInfo(Variant::FLOAT, name + "/position", PROPERTY_HINT_NONE, "", usage));
		p_list->push_back(PropertyInfo(Variant::NODE_PATH, name + "/node", PROPERTY_HINT_NODE_PATH_TO_EDITED_NODE, "", usage));
	}
}

bool RopePositions::_rp_property_can_revert(const StringName &p_name) const {
	if (p_name.begins_with(POSITION_PREFIX)) {
		return true;
	}

	return false;
}

bool RopePositions::_rp_property_get_revert(const StringName &p_name, Variant &r_property) const {
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

bool RopePositions::_rp_set(const StringName &p_name, const Variant &p_property) {
	if (p_name.begins_with(POSITION_PREFIX)) {
		Pair<uint64_t, String> subprop = _get_propname_with_index(p_name);
		uint64_t idx = subprop.first;
		String sub_name = subprop.second;
		if (sub_name == "position") {
			_set_position(idx, p_property);
		} else if (sub_name == "node") {
			_set_nodepath(idx, p_property);
		} else {
			return false;
		}
		return true;
	}
	return false;
}

bool RopePositions::_rp_get(const StringName &p_name, Variant &r_property) const {
	if (p_name.begins_with(POSITION_PREFIX)) {
		Pair<uint64_t, String> subprop = _get_propname_with_index(p_name);
		uint64_t idx = subprop.first;
		String sub_name = subprop.second;
		if (sub_name == "position") {
			r_property = _get_position(idx);
		} else if (sub_name == "node") {
			r_property = _get_nodepath(idx);
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

void RopePositions::_set_count(uint64_t count) {
	if (count > MAX_POSITIONS) {
		print_error("RopePositions::_set_count: invalid count passed to set_position_count(), max_count:" + itos(MAX_POSITIONS));
		return;
	}

	if (_positions.size() != count) {
		_positions.resize(count);
		_notify_changed();
	}
}

uint64_t RopePositions::_get_count() const {
	return _positions.size();
}

void RopePositions::_set_position(uint64_t idx, float val) {
	if (idx >= _positions.size()) {
		print_error("RopePositions::_set_position: index out of range.");
		return;
	}

	Position p = _positions.get(idx);
	p.position = val;
	_positions.set(idx, p);

	_notify_changed();
}

float RopePositions::_get_position(uint64_t idx) const {
	if (idx >= _positions.size()) {
		print_error("RopePositions::_get_position: index out of range.");
		return -1.0;
	}

	return _positions.get(idx).position;
}

void RopePositions::_set_nodepath(uint64_t idx, const NodePath &val) {
	if (idx >= _positions.size()) {
		print_error("RopePositions::_set_nodepath: index out of range.");
		return;
	}

	Position p = _positions.get(idx);
	p.node = val;
	_positions.set(idx, p);

	_notify_changed();
}

NodePath RopePositions::_get_nodepath(uint64_t idx) const {
	if (idx >= _positions.size()) {
		print_error("RopePositions::_get_nodepath: index out of range.");
		return "error";
	}

	return _positions.get(idx).node;
}

void RopePositions::_set_spacing(Spacing val) {
	_spacing = val;
}

RopePositions::Spacing RopePositions::_get_spacing() const {
	return _spacing;
}

float RopePositions::_position_for_distance(float distance, float rope_length) const {
	if (rope_length == 0)
		return 0.0;

	float ret = 0.0;
	switch (_spacing) {
		case Scalar:
			ret = distance;
			break;
		case DistanceFromStart:
			ret = distance / rope_length;
			break;
		case DistanceFromEnd:
			ret = 1.0 - (distance / rope_length);
			break;
	}

	// clamp to the limits when the distance exceeds the rope length
	return CLAMP(ret, 0.0f, 1.0f);
}

float RopePositions::_distance_for_position(float position, float rope_length) const {
	if (rope_length == 0)
		return 0.0;

	switch (_spacing) {
		case Scalar:
			return position;
		case DistanceFromStart:
			return position * rope_length;
		case DistanceFromEnd:
			return rope_length - (position * rope_length);
	}

	return -1.0;
}