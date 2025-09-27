#include "rope_attachments_base.h"
#include "rope.h"

void RopeAttachmentsBase::_bind_methods() {
	GDVIRTUAL_BIND(get_count);
	GDVIRTUAL_BIND(get_position);
	GDVIRTUAL_BIND(get_nodepath);
}

uint64_t RopeAttachmentsBase::get_count(const Rope *rope) const {
	if (!GDVIRTUAL_IS_OVERRIDDEN(get_count)) {
		ERR_FAIL_V_MSG(0, "get_count must be overriden");
	} else {
		uint64_t ret_val;
		GDVIRTUAL_CALL(get_count, rope, ret_val);
		return ret_val;
	}
}

float RopeAttachmentsBase::get_position(uint64_t idx, const Rope *rope) const {
	if (!GDVIRTUAL_IS_OVERRIDDEN(get_position)) {
		ERR_FAIL_V_MSG(-1.0, "get_position must be overriden");
	} else {
		float ret_val;
		GDVIRTUAL_CALL(get_position, idx, rope, ret_val);
		return ret_val;
	}
}

NodePath RopeAttachmentsBase::get_nodepath(uint64_t idx, const Rope *rope) const {
	if (!GDVIRTUAL_IS_OVERRIDDEN(get_nodepath)) {
		ERR_FAIL_V_MSG(Transform3D(), "get_nodepath must be overriden");
	} else {
		NodePath ret_val;
		GDVIRTUAL_CALL(get_nodepath, idx, rope, ret_val);
		return ret_val;
	}
}
