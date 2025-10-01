#pragma once

#include "property_utils.h"
#include "rope_anchors_base.h"
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

class RopeAnchorPositions : public RopeAnchorsBase, public RopePositions {
	GDCLASS(RopeAnchorPositions, RopeAnchorsBase)

protected:
	static void _bind_methods();
	void _notify_changed() override;

	void _get_property_list(List<PropertyInfo> *p_list) const { _rp_get_property_list(p_list); }
	bool _property_can_revert(const StringName &p_name) const { return _rp_property_can_revert(p_name); }
	bool _property_get_revert(const StringName &p_name, Variant &r_property) const { return _rp_property_get_revert(p_name, r_property); }
	bool _set(const StringName &p_name, const Variant &p_property) { return _rp_set(p_name, p_property); }
	bool _get(const StringName &p_name, Variant &r_property) const { return _rp_get(p_name, r_property); }

public:
	RopeAnchorPositions() = default;
	virtual ~RopeAnchorPositions() = default;

	void set_position_count(uint64_t count) { _set_count(count); }
	uint64_t get_position_count() const { return _get_count(); }

	void set_spacing(Spacing val) { _set_spacing(val); }
	Spacing get_spacing() const { return _get_spacing(); }

	//GDVIRTUAL1RC(uint64_t, get_count, const Rope *);
	virtual uint64_t get_count(const Rope *rope) const override;

	// these return Scalar positions
	void set_position(uint64_t idx, float val, const Rope *rope);
	//GDVIRTUAL2RC(float, get_position, uint64_t, const Rope *);
	virtual float get_position(uint64_t idx, const Rope *rope) const override;

	//GDVIRTUAL2RC(Transform3D, get_transform, uint64_t, const Rope *);
	virtual Transform3D get_transform(uint64_t idx, const Rope *rope) const override;

	// via RopePositions
	void set_nodepath(uint64_t idx, const NodePath &val, const Rope *rope);
	NodePath get_nodepath(uint64_t idx, const Rope *rope) const;
};