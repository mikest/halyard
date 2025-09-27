#include "rope_attachment_positions.h"
#include "rope.h"

void RopeAttachmentPositions::_bind_methods() {
	BIND_ENUM_CONSTANT(Scalar);
	BIND_ENUM_CONSTANT(DistanceFromStart);
	BIND_ENUM_CONSTANT(DistanceFromEnd);

	EXPORT_PROPERTY_ENUM(spacing, POSITIONS_HINT, RopeAttachmentPositions);
	EXPORT_PROPERTY(Variant::INT, position_count, RopeAttachmentPositions);

	ClassDB::bind_method(D_METHOD("get_count", "rope"), &RopeAttachmentPositions::get_count);

	// NOTE: the resource loader will use the _set_xxx calls on the RopePositions class, not these calls
	ClassDB::bind_method(D_METHOD("set_position", "index", "position", "rope"), &RopeAttachmentPositions::set_position);
	ClassDB::bind_method(D_METHOD("get_position", "index", "rope"), &RopeAttachmentPositions::get_position);

	ClassDB::bind_method(D_METHOD("set_nodepath", "index", "node", "rope"), &RopeAttachmentPositions::set_nodepath);
	ClassDB::bind_method(D_METHOD("get_nodepath", "index", "rope"), &RopeAttachmentPositions::get_nodepath);

	GDVIRTUAL_BIND(get_count, "rope");
	GDVIRTUAL_BIND(get_position, "idx", "rope");
	GDVIRTUAL_BIND(get_nodepath, "idx", "rope");
}

void RopeAttachmentPositions::_notify_changed() {
	notify_property_list_changed();
}

uint64_t RopeAttachmentPositions::get_count(const Rope *rope) const {
	ERR_FAIL_NULL_V_MSG(rope, 0, "Missing rope");
	return get_position_count();
}

void RopeAttachmentPositions::set_position(uint64_t idx, float val, const Rope *rope) {
	ERR_FAIL_NULL_MSG(rope, "Missing rope");
	_set_position(idx, _distance_for_position(val, rope->get_rope_length()));
}

float RopeAttachmentPositions::get_position(uint64_t idx, const Rope *rope) const {
	ERR_FAIL_NULL_V_MSG(rope, -1.0, "Missing rope");
	return _position_for_distance(_get_position(idx), rope->get_rope_length());
}

void RopeAttachmentPositions::set_nodepath(uint64_t idx, const NodePath &val, const Rope *rope) {
	ERR_FAIL_NULL_MSG(rope, "Missing rope");
	_set_nodepath(idx, val);
}

NodePath RopeAttachmentPositions::get_nodepath(uint64_t idx, const Rope *rope) const {
	ERR_FAIL_NULL_V_MSG(rope, NodePath(), "Missing rope");
	return _get_nodepath(idx);
}