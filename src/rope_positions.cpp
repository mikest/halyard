#include "rope_positions.h"

void RopePositions::_bind_methods() {
#define STR(x) #x
#define EXPORT_PROPERTY(m_type, m_property)                                                                   \
	ClassDB::bind_method(D_METHOD(STR(set_##m_property), STR(m_property)), &RopePositions::set_##m_property); \
	ClassDB::bind_method(D_METHOD(STR(get_##m_property)), &RopePositions::get_##m_property);                  \
	ADD_PROPERTY(PropertyInfo(m_type, #m_property), STR(set_##m_property), STR(get_##m_property))

	EXPORT_PROPERTY(Variant::INT, position_count);

	ClassDB::bind_method(D_METHOD("set_position", "surface_index", "rotation"), &RopePositions::set_position);
	ClassDB::bind_method(D_METHOD("get_position", "surface_index"), &RopePositions::get_position);

#undef EXPORT_PROPERTY
#undef STR
}

void RopePositions::_get_property_list(List<PropertyInfo> *p_list) const {
	for (uint64_t idx = 0; idx < _positions.size(); ++idx) {
		uint32_t usage = PROPERTY_USAGE_DEFAULT;
		String name = "position_" + itos(idx);

		p_list->push_back(PropertyInfo(Variant::NIL, name.capitalize(), PROPERTY_HINT_NONE, name + "/", PROPERTY_USAGE_GROUP));
		p_list->push_back(PropertyInfo(Variant::FLOAT, name + "/position", PROPERTY_HINT_RANGE, "0.0,1.0,0.01", usage));
		p_list->push_back(PropertyInfo(Variant::NODE_PATH, name + "/node", PROPERTY_HINT_NODE_PATH_TO_EDITED_NODE, "", usage));
	}
}

bool RopePositions::_property_can_revert(const StringName &p_name) const {
	if (p_name.begins_with("position_")) {
		return true;
	}

	return false;
}

bool RopePositions::_property_get_revert(const StringName &p_name, Variant &r_property) const {
	if (p_name.begins_with("position_")) {
		Pair<uint64_t, String> subprop = _decode_dynamic_propname(p_name);
		uint64_t surf_idx = subprop.first;
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
	if (p_name.begins_with("position_")) {
		Pair<uint64_t, String> subprop = _decode_dynamic_propname(p_name);
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
	if (p_name.begins_with("position_")) {
		Pair<uint64_t, String> subprop = _decode_dynamic_propname(p_name);
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

Pair<uint64_t, String> RopePositions::_decode_dynamic_propname(const StringName &p_name) const {
	PackedStringArray prop_path = p_name.split("/");
	if (prop_path.size() != 2) {
		return Pair<uint64_t, String>();
	}

	uint64_t surf_idx = prop_path[0].trim_prefix("position_").to_int();
	String sub_name = prop_path[1];
	return Pair(surf_idx, sub_name);
}