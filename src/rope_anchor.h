/* Copyright (c) M. Estee. MIT License.

RopeAnchor is a class for representing different arnchor behaviors.
These include common ways ropes are attached or travel through points in space.

This utility class can act on its own, or it can act on a parent object when it
has been set as a child of that object.

*/

#pragma once

#include "property_utils.h"
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/rigid_body3d.hpp>
#include <godot_cpp/core/binder_common.hpp>
#include <godot_cpp/core/gdvirtual.gen.inc>

using namespace godot;

enum AnchorBehavior {
	FREE = -1, // Rope is unattached to this anchor
	ANCHORED = 0, // Rope is attached to anchor and can not move it. (Not towing or guided.)
	TOWING = 1, // If anchors is a child of a RigidBody3D rope will pull on it, otherwise acts like ANCHORED.
	GUIDED = 2, // The rope travels freely through this point, but the rope cannot move the anchor.
	SLIDING = 3, // If parent is a RigidBody, rope travels through like guided, and pulls on parent. (Towing and guided.)
};

// A node that serves as a list of anchor positions for a section of rope.
// Each anchor position shares the same behavior.
// When parented to a RigidBody3D, it can apply forces at its position.
class RopeAnchor : public Node3D {
	GDCLASS(RopeAnchor, Node3D)

private:
	int _behavior = ANCHORED;

protected:
	static void _bind_methods();

public:
	RopeAnchor() = default;
	virtual ~RopeAnchor() override = default;

	PROPERTY_GET_SET(int, behavior, {})

	// Returns the parent node if it is a RigidBody3D, otherwise nullptr.
	RigidBody3D *get_parent_rigid_body() const;

	// returns the number of positions this anchor provides. Minimum is 1.
	GDVIRTUAL0RC(int, get_particle_count);
	virtual int get_particle_count() const;

	// returns the position of the anchor in local space.
	GDVIRTUAL1RC(Vector3, get_particle_position, int);
	virtual Vector3 get_particle_position(int idx=0) const;

	// Applies a force to the parent RigidBody3D at this anchor's global position.
	void apply_force(const Vector3 &p_force);
};

VARIANT_ENUM_CAST(AnchorBehavior);
