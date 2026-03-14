#pragma once

#include "property_utils.h"
#include "rope_anchors_base.h"
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

class RopeAnchorPositions : public RopeAnchorsBase {
	GDCLASS(RopeAnchorPositions, RopeAnchorsBase)

public:
	enum Spacing {
		Scalar = 0,
		DistanceFromStart = 1,
		DistanceFromEnd = 2,
	};

protected:
	struct Position {
		float position = 0.0;
		NodePath node = "";
	};

	struct Anchor {
		float position = 0.0;
		NodePath node_path = "";
		AnchorBehavior behavior = AnchorBehavior::ANCHORED;
		Anchor() = default;
	};

	Vector<Position> _positions;
	LocalVector<Anchor> _anchors;
	Spacing _spacing = Scalar;

	Pair<uint64_t, String> _get_propname_with_index(const StringName &p_name) const;

	// Internal accessors; these call _notify_changed on mutations
	void _set_count(uint64_t count);
	uint64_t _get_count() const;

	void _set_position(uint64_t idx, float val);
	float _get_position(uint64_t idx) const;
	void _set_nodepath(uint64_t idx, const NodePath &val);
	NodePath _get_nodepath(uint64_t idx) const;

	void _set_anchor_position(int idx, float val);
	float _get_anchor_position(int idx) const;
	void _set_anchor_node(int idx, NodePath val);
	NodePath _get_anchor_node(int idx) const;
	void _set_anchor_behavior(int idx, AnchorBehavior val);
	AnchorBehavior _get_anchor_behavior(int idx) const;

	void sort_anchors();
	Callable _get_sort_anchors_callable() const;

	void _set_spacing(Spacing val);
	Spacing _get_spacing() const;

	// Convert between scalar [0,1] position and distance, depending on the current spacing mode
	float _position_for_distance(float distance, float rope_length) const;
	float _distance_for_position(float position, float rope_length) const;

	static void _bind_methods();
	void _notify_changed();

	// Dynamic Property List
	void _get_property_list(List<PropertyInfo> *p_list) const;
	void _validate_dynamic_prop(PropertyInfo &p_property) const;
	bool _property_can_revert(const StringName &p_name) const;
	bool _property_get_revert(const StringName &p_name, Variant &r_property) const;
	bool _set(const StringName &p_name, const Variant &p_property);
	bool _get(const StringName &p_name, Variant &r_property) const;

public:
	RopeAnchorPositions() = default;
	virtual ~RopeAnchorPositions() = default;

	void set_anchor_count(int count);
	int get_anchor_count() const;
	void clear_anchors();

	void set_position_count(uint64_t count) { _set_count(count); }
	uint64_t get_position_count() const { return _get_count(); }

	void set_spacing(Spacing val) { _set_spacing(val); }
	Spacing get_spacing() const { return _get_spacing(); }

	virtual uint64_t get_count(const Rope *rope) const override;

	// these return Scalar positions
	void set_position(uint64_t idx, float val, const Rope *rope);
	virtual float get_position(uint64_t idx, const Rope *rope) const override;

	virtual Transform3D get_transform(uint64_t idx, const Rope *rope) const override;

	//GDVIRTUAL2RC(RopeBehavior, get_behavior, uint64_t, const Rope *);
	virtual AnchorBehavior get_behavior(uint64_t idx, const Rope *rope) const override;

	//GDVIRTUAL2RC(Node3D*, get_anchor_parent, uint64_t, const Rope *);
	virtual Node3D *get_anchor_parent(uint64_t idx, const Rope *rope) const override;

	// via RopePositions
	void set_anchor_node(uint64_t idx, const Node3D *val, const Rope *rope);
	Node3D *get_anchor_node(uint64_t idx, const Rope *rope) const;

	void set_nodepath(uint64_t idx, const NodePath &val, const Rope *rope);
	NodePath get_nodepath(uint64_t idx, const Rope *rope) const;
};

VARIANT_ENUM_CAST(RopeAnchorPositions::Spacing)
