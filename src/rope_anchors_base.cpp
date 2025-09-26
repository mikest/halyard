#include "rope_anchors_base.h"
#include "rope.h"

void RopeAnchorsBase::_bind_methods() {
	GDVIRTUAL_BIND(get_anchor_count);
	GDVIRTUAL_BIND(get_anchor_position);
	GDVIRTUAL_BIND(get_anchor_transform);
}

uint64_t RopeAnchorsBase::get_anchor_count(const Rope *rope) const {
	if (!GDVIRTUAL_IS_OVERRIDDEN(get_anchor_count)) {
		ERR_FAIL_V_MSG(0, "get_anchor_count must be overriden");
	} else {
		uint64_t ret_val;
		GDVIRTUAL_CALL(get_anchor_count, rope, ret_val);
		return ret_val;
	}
}

float RopeAnchorsBase::get_anchor_position(uint64_t idx, const Rope *rope) const {
	if (!GDVIRTUAL_IS_OVERRIDDEN(get_anchor_position)) {
		ERR_FAIL_V_MSG(-1.0, "get_anchor_position must be overriden");
	} else {
		float ret_val;
		GDVIRTUAL_CALL(get_anchor_position, idx, rope, ret_val);
		return ret_val;
	}
}

Transform3D RopeAnchorsBase::get_anchor_transform(uint64_t idx, const Rope *rope) const {
	if (!GDVIRTUAL_IS_OVERRIDDEN(get_anchor_transform)) {
		ERR_FAIL_V_MSG(Transform3D(), "get_anchor_transform must be overriden");
	} else {
		Transform3D ret_val;
		GDVIRTUAL_CALL(get_anchor_transform, idx, rope, ret_val);
		return ret_val;
	}
}
