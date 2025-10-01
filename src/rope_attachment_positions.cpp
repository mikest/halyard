#include "rope_attachment_positions.h"
#include "rope.h"

void RopeAttachmentPositions::_bind_methods() {
	BIND_ENUM_CONSTANT(Scalar);
	BIND_ENUM_CONSTANT(DistanceFromStart);
	BIND_ENUM_CONSTANT(DistanceFromEnd);

	EXPORT_PROPERTY_ENUM(spacing, POSITIONS_HINT, RopeAttachmentPositions);
	EXPORT_PROPERTY(Variant::INT, position_count, RopeAttachmentPositions);

	ClassDB::bind_method(D_METHOD("get_count", "rope"), &RopeAttachmentPositions::get_count);
	// GDVIRTUAL_BIND(get_count, "rope");

	// NOTE: the resource loader will use the _set_xxx calls on the RopePositions class, not these calls
	ClassDB::bind_method(D_METHOD("set_position", "index", "position", "rope"), &RopeAttachmentPositions::set_position);
	ClassDB::bind_method(D_METHOD("get_position", "index", "rope"), &RopeAttachmentPositions::get_position);
	// GDVIRTUAL_BIND(get_position, "idx", "rope");

	ClassDB::bind_method(D_METHOD("set_nodepath", "index", "node", "rope"), &RopeAttachmentPositions::set_nodepath);
	ClassDB::bind_method(D_METHOD("get_nodepath", "index", "rope"), &RopeAttachmentPositions::get_nodepath);
	// GDVIRTUAL_BIND(get_nodepath, "idx", "rope");
}

void RopeAttachmentPositions::_notify_changed() {
	notify_property_list_changed();
}

uint64_t RopeAttachmentPositions::get_count(const Rope *rope) const {
	uint64_t ret_val = 0;
	ERR_FAIL_NULL_V_MSG(rope, ret_val, "Missing rope");

	if (GDVIRTUAL_IS_OVERRIDDEN(get_count)) {
		GDVIRTUAL_CALL(get_count, rope, ret_val);
	} else if (rope) {
		ret_val = get_position_count();
	}

	return ret_val;
}

void RopeAttachmentPositions::set_position(uint64_t idx, float val, const Rope *rope) {
	ERR_FAIL_NULL_MSG(rope, "Missing rope");
	_set_position(idx, _distance_for_position(val, rope->get_rope_length()));
}

float RopeAttachmentPositions::get_position(uint64_t idx, const Rope *rope) const {
	float ret_val = -1.0;
	ERR_FAIL_NULL_V_MSG(rope, ret_val, "Missing rope");

	if (GDVIRTUAL_IS_OVERRIDDEN(get_position)) {
		GDVIRTUAL_CALL(get_position, idx, rope, ret_val);
	} else if (rope) {
		ret_val = _position_for_distance(_get_position(idx), rope->get_rope_length());
	}

	return ret_val;
}

void RopeAttachmentPositions::set_nodepath(uint64_t idx, const NodePath &val, const Rope *rope) {
	ERR_FAIL_NULL_MSG(rope, "Missing rope");
	_set_nodepath(idx, val);
}

NodePath RopeAttachmentPositions::get_nodepath(uint64_t idx, const Rope *rope) const {
	NodePath ret_val;
	ERR_FAIL_NULL_V_MSG(rope, ret_val, "Missing rope");

	if (GDVIRTUAL_IS_OVERRIDDEN(get_nodepath)) {
		GDVIRTUAL_CALL(get_nodepath, idx, rope, ret_val);
	} else if (rope) {
		ret_val = _get_nodepath(idx);
	}

	return ret_val;
}