#include "rope_attachments_base.h"
#include "rope.h"

void RopeAttachmentsBase::_bind_methods() {
	GDVIRTUAL_BIND(get_attachment_count);
	GDVIRTUAL_BIND(get_attachment_position);
	GDVIRTUAL_BIND(get_attachment_nodepath);
}

uint64_t RopeAttachmentsBase::get_attachment_count(const Rope *rope) const {
	if (!GDVIRTUAL_IS_OVERRIDDEN(get_attachment_count)) {
		ERR_FAIL_V_MSG(0, "get_attachment_count must be overriden");
	} else {
		uint64_t ret_val;
		GDVIRTUAL_CALL(get_attachment_count, rope, ret_val);
		return ret_val;
	}
}

float RopeAttachmentsBase::get_attachment_position(uint64_t idx, const Rope *rope) const {
	if (!GDVIRTUAL_IS_OVERRIDDEN(get_attachment_position)) {
		ERR_FAIL_V_MSG(-1.0, "get_attachment_position must be overriden");
	} else {
		float ret_val;
		GDVIRTUAL_CALL(get_attachment_position, idx, rope, ret_val);
		return ret_val;
	}
}

NodePath RopeAttachmentsBase::get_attachment_nodepath(uint64_t idx, const Rope *rope) const {
	if (!GDVIRTUAL_IS_OVERRIDDEN(get_attachment_nodepath)) {
		ERR_FAIL_V_MSG(Transform3D(), "get_attachment_nodepath must be overriden");
	} else {
		NodePath ret_val;
		GDVIRTUAL_CALL(get_attachment_nodepath, idx, rope, ret_val);
		return ret_val;
	}
}
