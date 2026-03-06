#include "rope_anchor_positions.h"
#include "rope.h"

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