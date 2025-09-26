#include "rope_anchor_positions.h"
#include "rope.h"

void RopeAnchorPositions::_bind_methods() {
	BIND_ENUM_CONSTANT(Scalar);
	BIND_ENUM_CONSTANT(DistanceFromStart);
	BIND_ENUM_CONSTANT(DistanceFromEnd);

	EXPORT_PROPERTY_ENUM(spacing, POSITIONS_HINT, RopeAnchorPositions);

	EXPORT_PROPERTY(Variant::INT, count, RopeAnchorPositions);

	ClassDB::bind_method(D_METHOD("set_position", "index", "position"), &RopeAnchorPositions::set_anchor_position);
	ClassDB::bind_method(D_METHOD("get_position", "index"), &RopeAnchorPositions::get_anchor_position);

	ClassDB::bind_method(D_METHOD("set_node", "index", "node"), &RopeAnchorPositions::set_anchor_node);
	ClassDB::bind_method(D_METHOD("get_node", "index"), &RopeAnchorPositions::get_anchor_node);
}

void RopeAnchorPositions::_notify_changed() {
	notify_property_list_changed();
}

void RopeAnchorPositions::set_anchor_position(uint64_t idx, float val, const Rope *rope) {
	ERR_FAIL_NULL_MSG(rope, "Missing rope");
	_set_position(idx, _distance_for_position(val, rope->get_rope_length()));
}

float RopeAnchorPositions::get_anchor_position(uint64_t idx, const Rope *rope) const {
	ERR_FAIL_NULL_V_MSG(rope, -1.0, "Missing rope");
	return _position_for_distance(_get_position(idx), rope->get_rope_length());
}

Transform3D RopeAnchorPositions::get_anchor_transform(uint64_t idx, const Rope *rope) const {
	Transform3D ret_val;

	if (GDVIRTUAL_IS_OVERRIDDEN(get_anchor_transform)) {
		GDVIRTUAL_CALL(get_anchor_transform, idx, rope, ret_val);
	} else if (rope) {
		const NodePath &path = get_anchor_node(idx);
		Node3D *node = cast_to<Node3D>(rope->get_node_or_null(path));
		if (node && node->is_visible()) {
			ret_val = node->get_global_transform();
		}
	}

	return ret_val;
}

#pragma endregion