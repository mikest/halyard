#include "rope_anchor_positions.h"
#include "rope.h"
#include <godot_cpp/variant/callable.hpp>

#define POSITION_PREFIX "position_"
#define POSITIONS_HINT "Scalar,DistanceFromStart,DistanceFromEnd"
#define BEHAVIOR_HINT "Free:-1,Anchored:0,Towing:1,Guided:2,Sliding:3"

// performance above this value is going to be... disappointing.
static const int MAX_POSITIONS = 1000;

// PROPERTY_HINT_TOOL_BUTTON may not be in older godot-cpp builds; define it by value.
#ifndef PROPERTY_HINT_TOOL_BUTTON
static constexpr PropertyHint PROPERTY_HINT_TOOL_BUTTON = (PropertyHint)39;
#endif

#pragma region Dynamic Properties

// Build up a dynamic list of properties for each element in the position vector.
void RopeAnchorPositions::_get_property_list(List<PropertyInfo> *p_list) const {
	ERR_FAIL_NULL(p_list);

	for (uint64_t idx = 0; idx < _positions.size(); ++idx) {
		String name = POSITION_PREFIX + itos(idx);

		p_list->push_back(PropertyInfo(Variant::NIL, name.capitalize(), PROPERTY_HINT_NONE, name + "/", PROPERTY_USAGE_GROUP));
		p_list->push_back(PropertyInfo(Variant::FLOAT, name + "/position", PROPERTY_HINT_NONE, ""));
		p_list->push_back(PropertyInfo(Variant::NODE_PATH, name + "/node", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Node3D,RopeAnchor,RigidBody3D"));
	}

	// new layout
	LocalVector<PropertyInfo> props;
	for (uint32_t i = 0; i < _anchors.size(); i++) {
		String path = "anchors/" + itos(i) + "/";
		p_list->push_back(PropertyInfo(Variant::FLOAT, path + "position"));
		p_list->push_back(PropertyInfo(Variant::NODE_PATH, path + "node_path", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "RopeAnchor,RigidBody3D"));
		p_list->push_back(PropertyInfo(Variant::INT, path + "behavior", PROPERTY_HINT_ENUM, BEHAVIOR_HINT));
	}
}


bool RopeAnchorPositions::_property_can_revert(const StringName &p_name) const {
	if (p_name.begins_with(POSITION_PREFIX)) {
		return true;
	}

	return false;
}

bool RopeAnchorPositions::_property_get_revert(const StringName &p_name, Variant &r_property) const {
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

bool RopeAnchorPositions::_set(const StringName &p_name, const Variant &p_property) {
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

	String path = p_name;
	if (path.begins_with("anchors/")) {
		int which = path.get_slicec('/', 1).to_int();
		String what = path.get_slicec('/', 2);
		ERR_FAIL_INDEX_V(which, (int)_anchors.size(), false);

		WARN_PRINT("_set:" + path);

		if (what == "position") {
			_set_anchor_position(which, (float)p_property);
		} else if (what == "node_path") {
			_set_anchor_node(which, (NodePath)p_property);
		} else if (what == "behavior") {
			_set_anchor_behavior(which, (AnchorBehavior)(int)p_property);
		} else {
			return false;
		}
		return true;
	}

	return false;
}

bool RopeAnchorPositions::_get(const StringName &p_name, Variant &r_property) const {
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

	// new layout
	String path = p_name;
	if (path.begins_with("anchors/")) {
		int which = path.get_slicec('/', 1).to_int();
		String what = path.get_slicec('/', 2);
		ERR_FAIL_INDEX_V(which, (int)_anchors.size(), false);

		if (what == "position") {
			r_property = _get_anchor_position(which);
		} else if (what == "node_path") {
			r_property = (NodePath)_get_anchor_node(which);
		} else if (what == "behavior") {
			r_property = (int)_get_anchor_behavior(which);
		} else {
			return false;
		}
		return true;
	}

	return false;
}

#pragma endregion

#pragma region Anchors
void RopeAnchorPositions::set_anchor_count(int count) {
	WARN_PRINT("RopeAnchorPositions::set_anchor_count: " + itos(count));
	_anchors.resize(count);
	notify_property_list_changed();
}

int RopeAnchorPositions::get_anchor_count() const {
	return _anchors.size();
}

void RopeAnchorPositions::clear_anchors() {
	_anchors.clear();
	notify_property_list_changed();
}

void RopeAnchorPositions::_set_anchor_behavior(int idx, AnchorBehavior val) {
	if (idx >= (int)_anchors.size()) {
		print_error("RopeAnchorPositions::_set_anchor_behavior: index out of range.");
		return;
	}
	_anchors[idx].behavior = val;
	notify_property_list_changed();
}

AnchorBehavior RopeAnchorPositions::_get_anchor_behavior(int idx) const {
	if (idx >= (int)_anchors.size()) {
		print_error("RopeAnchorPositions::_get_anchor_behavior: index out of range.");
		return AnchorBehavior::ANCHORED;
	}
	return _anchors[idx].behavior;
}

void RopeAnchorPositions::_set_anchor_position(int idx, float val) {
	if (idx >= (int)_anchors.size()) {
		print_error("RopeAnchorPositions::_set_anchor_position: index out of range.");
		return;
	}
	_anchors[idx].position = val;
	notify_property_list_changed();
}

float RopeAnchorPositions::_get_anchor_position(int idx) const {
	if (idx >= (int)_anchors.size()) {
		print_error("RopeAnchorPositions::_get_anchor_position: index out of range.");
		return -1.0;
	}
	return _anchors[idx].position;
}

void RopeAnchorPositions::_set_anchor_node(int idx, NodePath val) {
	if (idx >= (int)_anchors.size()) {
		print_error("RopeAnchorPositions::_set_anchor_node: index out of range.");
		return;
	}
	_anchors[idx].node_path = val;
	notify_property_list_changed();
}

NodePath RopeAnchorPositions::_get_anchor_node(int idx) const {
	if (idx >= (int)_anchors.size()) {
		print_error("RopeAnchorPositions::_get_anchor_node: index out of range.");
		return "";
	}
	return _anchors[idx].node_path;
}

#pragma endregion

#pragma region Property Helpers

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

#if 1 // Array binding is not included in GDExtension, so replicate the calls here.
#define ADD_ARRAY_COUNT(m_label, m_count_property, m_count_property_setter, m_count_property_getter, m_prefix) ClassDB_add_property_array_count(get_class_static(), m_label, m_count_property, StringName(m_count_property_setter), StringName(m_count_property_getter), m_prefix)

inline void ClassDB_add_property_array_count(const StringName &p_class, const String &p_label, const StringName &p_count_property, const StringName &p_count_setter, const StringName &p_count_getter, const String &p_array_element_prefix, uint32_t p_count_usage = PROPERTY_USAGE_DEFAULT) {
	ClassDB::add_property(p_class, PropertyInfo(Variant::INT, p_count_property, PROPERTY_HINT_NONE, "", p_count_usage | PROPERTY_USAGE_ARRAY, vformat("%s,%s", p_label, p_array_element_prefix)), p_count_setter, p_count_getter);
}
#endif

void RopeAnchorPositions::_bind_methods() {
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

	ClassDB::bind_method(D_METHOD("set_anchor_count", "count"), &RopeAnchorPositions::set_anchor_count);
	ClassDB::bind_method(D_METHOD("get_anchor_count"), &RopeAnchorPositions::get_anchor_count);
	ClassDB::bind_method(D_METHOD("clear_anchors"), &RopeAnchorPositions::clear_anchors);

	// Properties
	BIND_ENUM_CONSTANT(Scalar);
	BIND_ENUM_CONSTANT(DistanceFromStart);
	BIND_ENUM_CONSTANT(DistanceFromEnd);
	EXPORT_PROPERTY_ENUM(spacing, POSITIONS_HINT, RopeAnchorPositions);

#if 0
	ClassDB::bind_method(D_METHOD("_get_sort_anchors_callable"), &RopeAnchorPositions::_get_sort_anchors_callable);
	ClassDB::add_property(get_class_static(), PropertyInfo(Variant::CALLABLE, "sort_anchors_btn", PROPERTY_HINT_TOOL_BUTTON, "Sort Anchors,Sort", PROPERTY_USAGE_EDITOR), "", "_get_sort_anchors_callable");
#endif

	ADD_ARRAY_COUNT("Anchors", "anchor_count", "set_anchor_count", "get_anchor_count", "anchors/");

	ClassDB::bind_method(D_METHOD("sort_anchors"), &RopeAnchorPositions::sort_anchors);

	EXPORT_PROPERTY(Variant::INT, position_count, RopeAnchorPositions);
}

#pragma endregion

void RopeAnchorPositions::_notify_changed() {
	notify_property_list_changed();
}

void RopeAnchorPositions::sort_anchors() {
	struct AnchorByPosition {
		bool operator()(const Anchor &a, const Anchor &b) const {
			return a.position < b.position;
		}
	};
	_anchors.sort_custom<AnchorByPosition>();
	_notify_changed();
}

Callable RopeAnchorPositions::_get_sort_anchors_callable() const {
	return Callable(const_cast<RopeAnchorPositions *>(this), "sort_anchors");
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
		return Object::cast_to<Node3D>(rope->get_node_or_null(_get_nodepath(idx)));
		
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
		}/* else if (idx < (uint64_t)_anchors.size()) {
			ret_val = _get_anchor_behavior((int)idx);
		}*/
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