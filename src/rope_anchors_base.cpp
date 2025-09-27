#include "rope_anchors_base.h"
#include "rope.h"

void RopeAnchorsBase::_bind_methods() {
	GDVIRTUAL_BIND(get_count, "rope");
	GDVIRTUAL_BIND(get_position, "idx", "rope");
	GDVIRTUAL_BIND(get_transform, "idx", "rope");
}

uint64_t RopeAnchorsBase::get_count(const Rope *rope) const {
	if (!GDVIRTUAL_IS_OVERRIDDEN(get_count)) {
		ERR_FAIL_V_MSG(0, "get_count must be overriden");
	} else {
		uint64_t ret_val;
		GDVIRTUAL_CALL(get_count, rope, ret_val);
		return ret_val;
	}
}

float RopeAnchorsBase::get_position(uint64_t idx, const Rope *rope) const {
	if (!GDVIRTUAL_IS_OVERRIDDEN(get_position)) {
		ERR_FAIL_V_MSG(-1.0, "get_position must be overriden");
	} else {
		float ret_val;
		GDVIRTUAL_CALL(get_position, idx, rope, ret_val);
		return ret_val;
	}
}

Transform3D RopeAnchorsBase::get_transform(uint64_t idx, const Rope *rope) const {
	if (!GDVIRTUAL_IS_OVERRIDDEN(get_transform)) {
		ERR_FAIL_V_MSG(Transform3D(), "get_transform must be overriden");
	} else {
		Transform3D ret_val;
		GDVIRTUAL_CALL(get_transform, idx, rope, ret_val);
		return ret_val;
	}
}
