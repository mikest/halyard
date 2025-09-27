#include "rope_anchor_positions.h"
#include "rope.h"

void RopeAnchorPositions::_bind_methods() {
	BIND_ENUM_CONSTANT(Scalar);
	BIND_ENUM_CONSTANT(DistanceFromStart);
	BIND_ENUM_CONSTANT(DistanceFromEnd);

	EXPORT_PROPERTY_ENUM(spacing, POSITIONS_HINT, RopeAnchorPositions);
	EXPORT_PROPERTY(Variant::INT, position_count, RopeAnchorPositions);

	ClassDB::bind_method(D_METHOD("get_count", "rope"), &RopeAnchorPositions::get_count);

	ClassDB::bind_method(D_METHOD("set_position", "index", "position", "rope"), &RopeAnchorPositions::set_position);
	ClassDB::bind_method(D_METHOD("get_position", "index", "rope"), &RopeAnchorPositions::get_position);

	ClassDB::bind_method(D_METHOD("set_nodepath", "index", "node", "rope"), &RopeAnchorPositions::set_nodepath);
	ClassDB::bind_method(D_METHOD("get_nodepath", "index", "rope"), &RopeAnchorPositions::get_nodepath);

	ClassDB::bind_method(D_METHOD("get_transform", "index", "rope"), &RopeAnchorPositions::get_transform);

	GDVIRTUAL_BIND(get_count, "rope");
	GDVIRTUAL_BIND(get_position, "idx", "rope");
	GDVIRTUAL_BIND(get_transform, "idx", "rope");
}

void RopeAnchorPositions::_notify_changed() {
	notify_property_list_changed();
}

uint64_t RopeAnchorPositions::get_count(const Rope *rope) const {
	ERR_FAIL_NULL_V_MSG(rope, 0, "Missing rope");
	return get_position_count();
}

void RopeAnchorPositions::set_nodepath(uint64_t idx, const NodePath &val, const Rope *rope) {
	ERR_FAIL_NULL_MSG(rope, "Missing rope");
	_set_nodepath(idx, val);
}

NodePath RopeAnchorPositions::get_nodepath(uint64_t idx, const Rope *rope) const {
	ERR_FAIL_NULL_V_MSG(rope, NodePath(), "Missing rope");
	return _get_nodepath(idx);
}

void RopeAnchorPositions::set_position(uint64_t idx, float val, const Rope *rope) {
	ERR_FAIL_NULL_MSG(rope, "Missing rope");
	_set_position(idx, _distance_for_position(val, rope->get_rope_length()));
}

float RopeAnchorPositions::get_position(uint64_t idx, const Rope *rope) const {
	ERR_FAIL_NULL_V_MSG(rope, -1.0, "Missing rope");
	return _position_for_distance(_get_position(idx), rope->get_rope_length());
}

Transform3D RopeAnchorPositions::get_transform(uint64_t idx, const Rope *rope) const {
	Transform3D ret_val;

	ERR_FAIL_NULL_V_MSG(rope, ret_val, "Missing rope");

	if (GDVIRTUAL_IS_OVERRIDDEN(get_transform)) {
		GDVIRTUAL_CALL(get_transform, idx, rope, ret_val);
	} else if (rope) {
		const NodePath &path = get_nodepath(idx, rope);
		Node3D *node = cast_to<Node3D>(rope->get_node_or_null(path));
		if (node && node->is_visible()) {
			ret_val = node->get_global_transform();
		}
	}

	return ret_val;
}

#pragma endregion