#include "rope_anchor_positions.h"
#include "rope.h"

#define POSITION_PREFIX "position_"
#define POSITIONS_HINT "Scalar,DistanceFromStart,DistanceFromEnd"

// performance above this value is going to be... disappointing.
static const int MAX_POSITIONS = 1000;

#pragma region RopePositions functionality

// Build up a dynamic list of properties for each element in the position vector.
void RopeAnchorPositions::_rp_get_property_list(List<PropertyInfo> *p_list) const {
	for (uint64_t idx = 0; idx < _positions.size(); ++idx) {
		String name = POSITION_PREFIX + itos(idx);

		p_list->push_back(PropertyInfo(Variant::NIL, name.capitalize(), PROPERTY_HINT_NONE, name + "/", PROPERTY_USAGE_GROUP));
		p_list->push_back(PropertyInfo(Variant::FLOAT, name + "/position", PROPERTY_HINT_NONE, ""));
		p_list->push_back(PropertyInfo(Variant::NODE_PATH, name + "/node", PROPERTY_HINT_NODE_PATH_TO_EDITED_NODE, ""));
	}
}

bool RopeAnchorPositions::_rp_property_can_revert(const StringName &p_name) const {
	if (p_name.begins_with(POSITION_PREFIX)) {
		return true;
	}

	return false;
}

bool RopeAnchorPositions::_rp_property_get_revert(const StringName &p_name, Variant &r_property) const {
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

bool RopeAnchorPositions::_rp_set(const StringName &p_name, const Variant &p_property) {
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

bool RopeAnchorPositions::_rp_get(const StringName &p_name, Variant &r_property) const {
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

Pair<uint64_t, String> RopeAnchorPositions::_get_propname_with_index(const StringName &p_name) const {
	PackedStringArray prop_path = p_name.split("/");
	if (prop_path.size() != 2) {
		return Pair<uint64_t, String>();
	}

	uint64_t idx = prop_path[0].trim_prefix(POSITION_PREFIX).to_int();
	String sub_name = prop_path[1];
	return Pair(idx, sub_name);
}

void RopeAnchorPositions::_set_count(uint64_t count) {
	if (count > MAX_POSITIONS) {
		print_error("RopeAnchorPositions::_set_count: invalid count passed to set_position_count(), max_count:" + itos(MAX_POSITIONS));
		return;
	}

	if (_positions.size() != count) {
		_positions.resize(count);
		_notify_changed();
	}
}

uint64_t RopeAnchorPositions::_get_count() const {
	return _positions.size();
}

void RopeAnchorPositions::_set_position(uint64_t idx, float val) {
	if (idx >= _positions.size()) {
		print_error("RopeAnchorPositions::_set_position: index out of range.");
		return;
	}

	Position p = _positions.get(idx);
	p.position = val;
	_positions.set(idx, p);

	_notify_changed();
}

float RopeAnchorPositions::_get_position(uint64_t idx) const {
	if (idx >= _positions.size()) {
		print_error("RopeAnchorPositions::_get_position: index out of range.");
		return -1.0;
	}

	return _positions.get(idx).position;
}

void RopeAnchorPositions::_set_nodepath(uint64_t idx, const NodePath &val) {
	if (idx >= _positions.size()) {
		print_error("RopeAnchorPositions::_set_nodepath: index out of range.");
		return;
	}

	Position p = _positions.get(idx);
	p.node = val;
	_positions.set(idx, p);

	_notify_changed();
}

NodePath RopeAnchorPositions::_get_nodepath(uint64_t idx) const {
	if (idx >= _positions.size()) {
		print_error("RopeAnchorPositions::_get_nodepath: index out of range.");
		return "error";
	}

	return _positions.get(idx).node;
}

void RopeAnchorPositions::_set_spacing(Spacing val) {
	_spacing = val;
}

RopeAnchorPositions::Spacing RopeAnchorPositions::_get_spacing() const {
	return _spacing;
}

float RopeAnchorPositions::_position_for_distance(float distance, float rope_length) const {
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

float RopeAnchorPositions::_distance_for_position(float position, float rope_length) const {
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

#pragma endregion

#pragma region Bindings

void RopeAnchorPositions::_bind_methods() {
	BIND_ENUM_CONSTANT(Scalar);
	BIND_ENUM_CONSTANT(DistanceFromStart);
	BIND_ENUM_CONSTANT(DistanceFromEnd);

	EXPORT_PROPERTY_ENUM(spacing, POSITIONS_HINT, RopeAnchorPositions);
	EXPORT_PROPERTY(Variant::INT, position_count, RopeAnchorPositions);

	ClassDB::bind_method(D_METHOD("get_count", "rope"), &RopeAnchorPositions::get_count);
	// GDVIRTUAL_BIND(get_count, "rope");

	ClassDB::bind_method(D_METHOD("set_position", "index", "position", "rope"), &RopeAnchorPositions::set_position);
	ClassDB::bind_method(D_METHOD("get_position", "index", "rope"), &RopeAnchorPositions::get_position);
	// GDVIRTUAL_BIND(get_position, "idx", "rope");

	ClassDB::bind_method(D_METHOD("set_anchor_node", "index", "node", "rope"), &RopeAnchorPositions::set_anchor_node);
	ClassDB::bind_method(D_METHOD("get_anchor_node", "index", "rope"), &RopeAnchorPositions::get_anchor_node);

	ClassDB::bind_method(D_METHOD("get_transform", "index", "rope"), &RopeAnchorPositions::get_transform);
	ClassDB::bind_method(D_METHOD("get_behavior", "index", "rope"), &RopeAnchorPositions::get_behavior);
	ClassDB::bind_method(D_METHOD("get_anchor_parent", "index", "rope"), &RopeAnchorPositions::get_anchor_parent);
	// GDVIRTUAL_BIND(get_transform, "idx", "rope");
}

#pragma endregion

void RopeAnchorPositions::_notify_changed() {
	notify_property_list_changed();
}

uint64_t RopeAnchorPositions::get_count(const Rope *rope) const {
	uint64_t ret_val = 0;
	ERR_FAIL_NULL_V_MSG(rope, ret_val, "Missing rope");

	if (GDVIRTUAL_IS_OVERRIDDEN(get_count)) {
		GDVIRTUAL_CALL(get_count, rope, ret_val);
	} else if (rope) {
		ret_val = get_position_count();
	}

	return ret_val;
}

void RopeAnchorPositions::set_anchor_node(uint64_t idx, const Node3D *node, const Rope *rope) {
	ERR_FAIL_NULL_MSG(rope, "Missing rope");
	_set_nodepath(idx, node ? node->get_path() : NodePath());
}

Node3D *RopeAnchorPositions::get_anchor_node(uint64_t idx, const Rope *rope) const {
	ERR_FAIL_NULL_V_MSG(rope, nullptr, "Missing rope");

	// if we're in a subclass implementation there may be no nodes backing the anchors,
	// in which case we should just return null.
	if (idx >= _get_count())
		return nullptr;
	else
		return _get_node(idx, rope);
}

void RopeAnchorPositions::set_position(uint64_t idx, float val, const Rope *rope) {
	ERR_FAIL_NULL_MSG(rope, "Missing rope");
	_set_position(idx, _distance_for_position(val, rope->get_rope_length()));
}

float RopeAnchorPositions::get_position(uint64_t idx, const Rope *rope) const {
	float ret_val = -1.0;
	ERR_FAIL_NULL_V_MSG(rope, ret_val, "Missing rope");

	if (GDVIRTUAL_IS_OVERRIDDEN(get_position)) {
		GDVIRTUAL_CALL(get_position, idx, rope, ret_val);

	} else if (rope) {
		ret_val = _position_for_distance(_get_position(idx), rope->get_rope_length());
	}

	return ret_val;
}

Transform3D RopeAnchorPositions::get_transform(uint64_t idx, const Rope *rope) const {
	Transform3D ret_val;
	ERR_FAIL_NULL_V_MSG(rope, ret_val, "Missing rope");
	ERR_FAIL_COND_V_MSG(!rope->is_inside_tree(), ret_val, "Rope must be inside the scene tree to get anchor transforms.");

	if (GDVIRTUAL_IS_OVERRIDDEN(get_transform)) {
		GDVIRTUAL_CALL(get_transform, idx, rope, ret_val);

	} else if (rope) {
		Node3D *node = get_anchor_node(idx, rope);
		if (node) {
			ret_val = node->get_global_transform();
		}
	}

	return ret_val;
}

AnchorBehavior RopeAnchorPositions::get_behavior(uint64_t idx, const Rope *rope) const {
	AnchorBehavior ret_val = AnchorBehavior::ANCHORED;
	ERR_FAIL_NULL_V_MSG(rope, ret_val, "Missing rope");

	if (GDVIRTUAL_IS_OVERRIDDEN(get_behavior)) {
		GDVIRTUAL_CALL(get_behavior, idx, rope, ret_val);

	} else if (rope) {
		Node3D *node = get_anchor_node(idx, rope);
		RopeAnchor *anchor = Object::cast_to<RopeAnchor>(node);
		if (anchor) {
			ret_val = (AnchorBehavior)anchor->get_behavior();
		}
	}

	return ret_val;
}

Node3D *RopeAnchorPositions::get_anchor_parent(uint64_t idx, const Rope *rope) const {
	ERR_FAIL_NULL_V_MSG(rope, nullptr, "Missing rope");
	Node3D *ret_val = nullptr;

	if (GDVIRTUAL_IS_OVERRIDDEN(get_anchor_parent)) {
		GDVIRTUAL_CALL(get_anchor_parent, idx, rope, ret_val);

	} else if (rope) {
		Node3D *node = get_anchor_node(idx, rope);
		if (node) {
			ret_val = Object::cast_to<Node3D>(node->get_parent());
		}
	}

	return ret_val;
}

#pragma endregion