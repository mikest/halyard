/* Copyright (c) 2024-25 M. Estee

RopeAnchorBase is a base class for returning a set of
anchor positions and behaviors. The anchors can be based on node
or determined procedurally.

Anchors have an index and a position.
Position is the distance in meters along the rope.
Positive distances are measured from rope start, negative distances are measured from rope end.
If the distance is greater than the rope length the behavior will be assumed to be FREE.
If the anchor at a given index has a rigid_body, that rigid body may have forces applied to it.
If the anchor at a given index has node, that node will used to derive the transform of the anchor and the rigid_body.
*/

#pragma once

#include "property_utils.h"
#include "rope.h"
#include "rope_anchor.h"
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
class RopeAnchorsBase : public Resource {
	GDCLASS(RopeAnchorsBase, Resource)

protected:
	static void _bind_methods();

public:
	RopeAnchorsBase() = default;
	virtual ~RopeAnchorsBase() override = default;

	// return the number of rope anchors
	GDVIRTUAL1RC(uint64_t, get_count, const Rope *);
	virtual uint64_t get_count(const Rope *) const;

	// return the scalar positions the rope should anchor at.
	GDVIRTUAL2RC(float, get_position, uint64_t, const Rope *);
	virtual float get_position(uint64_t, const Rope *) const;

	// return the anchor transforms for the index
	// note that there may not be a node, but there will always be a transform
	GDVIRTUAL2RC(Transform3D, get_transform, uint64_t, const Rope *);
	virtual Transform3D get_transform(uint64_t, const Rope *) const;

	// return the behavior for the index, defaults to ANCHORED
	GDVIRTUAL2RC(AnchorBehavior, get_behavior, uint64_t, const Rope *);
	virtual AnchorBehavior get_behavior(uint64_t, const Rope *) const;

	// return the node of the anchor parent node (TOWING, PULLEY)
	// or nullptr if there is no anchor parent node for this position
	GDVIRTUAL2RC(Node3D *, get_anchor_parent, uint64_t, const Rope *);
	virtual Node3D *get_anchor_parent(uint64_t, const Rope *) const;
};
