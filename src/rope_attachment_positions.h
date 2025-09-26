#pragma once

#include "property_utils.h"
#include "rope_attachments_base.h"
#include "rope_positions.h"
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/classes/wrapped.hpp>
#include <godot_cpp/core/binder_common.hpp>
#include <godot_cpp/core/gdvirtual.gen.inc>
#include <godot_cpp/core/type_info.hpp>
#include <godot_cpp/templates/local_vector.hpp>
#include <godot_cpp/templates/pair.hpp>
#include <godot_cpp/variant/variant.hpp>

using namespace godot;

class RopeAttachmentPositions : public RopeAttachmentsBase, public RopePositions {
	GDCLASS(RopeAttachmentPositions, RopeAttachmentsBase)

protected:
	static void _bind_methods();
	void _notify_changed() override;

	void _get_property_list(List<PropertyInfo> *p_list) const { _rp_get_property_list(p_list); }
	bool _property_can_revert(const StringName &p_name) const { return _rp_property_can_revert(p_name); }
	bool _property_get_revert(const StringName &p_name, Variant &r_property) const { return _rp_property_get_revert(p_name, r_property); }
	bool _set(const StringName &p_name, const Variant &p_property) { return _rp_set(p_name, p_property); }
	bool _get(const StringName &p_name, Variant &r_property) const { return _rp_get(p_name, r_property); }

public:
	RopeAttachmentPositions() = default;
	virtual ~RopeAttachmentPositions() = default;

	void set_spacing(Spacing val) { _set_spacing(val); }
	Spacing get_spacing() const { return _get_spacing(); }

	void set_count(uint64_t count) { _set_count(count); }
	uint64_t get_count() const { return _get_count(); }
	uint64_t get_attachment_count(const Rope *rope = nullptr) const { return _get_count(); }

	// these return Scalar positions
	void set_attachment_position(uint64_t idx, float val, const Rope *rope);
	virtual float get_attachment_position(uint64_t idx, const Rope *rope) const;

	void set_attachment_nodepath(uint64_t idx, const NodePath &val) { _set_nodepath(idx, val); }
	virtual NodePath get_attachment_nodepath(uint64_t idx, const Rope *rope = nullptr) const { return _get_nodepath(idx); }
};
