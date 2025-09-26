#include "rope_attachment_positions.h"
#include "rope.h"

void RopeAttachmentPositions::_bind_methods() {
	BIND_ENUM_CONSTANT(Scalar);
	BIND_ENUM_CONSTANT(DistanceFromStart);
	BIND_ENUM_CONSTANT(DistanceFromEnd);

	EXPORT_PROPERTY_ENUM(spacing, POSITIONS_HINT, RopeAttachmentPositions);

	EXPORT_PROPERTY(Variant::INT, count, RopeAttachmentPositions);

	ClassDB::bind_method(D_METHOD("set_position", "index", "position"), &RopeAttachmentPositions::set_attachment_position);
	ClassDB::bind_method(D_METHOD("get_position", "index"), &RopeAttachmentPositions::get_attachment_position);

	ClassDB::bind_method(D_METHOD("set_node", "index", "node"), &RopeAttachmentPositions::set_attachment_nodepath);
	ClassDB::bind_method(D_METHOD("get_node", "index"), &RopeAttachmentPositions::get_attachment_nodepath);
}

void RopeAttachmentPositions::_notify_changed() {
	notify_property_list_changed();
}

void RopeAttachmentPositions::set_attachment_position(uint64_t idx, float val, const Rope *rope) {
	ERR_FAIL_NULL_MSG(rope, "Missing rope");
	_set_position(idx, _distance_for_position(val, rope->get_rope_length()));
}

float RopeAttachmentPositions::get_attachment_position(uint64_t idx, const Rope *rope) const {
	ERR_FAIL_NULL_V_MSG(rope, -1.0, "Missing rope");
	return _position_for_distance(_get_position(idx), rope->get_rope_length());
}
