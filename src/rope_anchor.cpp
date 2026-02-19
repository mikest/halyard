/* Copyright (c) M. Estee. MIT License. */

#include "rope_anchor.h"

void RopeAnchor::_bind_methods() {
	EXPORT_PROPERTY_ENUM(behavior, "Anchored,Towing,Guided", RopeAnchor);
	BIND_ENUM_CONSTANT(ANCHORED); // Rope can't move anchor.
	BIND_ENUM_CONSTANT(TOWING); // Rope pulls on rigid body at this point
	BIND_ENUM_CONSTANT(GUIDED); // Rope travels freely through this point, but rope cannot move it.

	ClassDB::bind_method(D_METHOD("get_parent_rigid_body"), &RopeAnchor::get_parent_rigid_body);
	ClassDB::bind_method(D_METHOD("apply_force", "force"), &RopeAnchor::apply_force);
}

RigidBody3D *RopeAnchor::get_parent_rigid_body() const {
	Node *parent = get_parent();
	if (parent == nullptr) {
		return nullptr;
	}
	return Object::cast_to<RigidBody3D>(parent);
}

void RopeAnchor::apply_force(const Vector3 &p_force) {
	RigidBody3D *body = get_parent_rigid_body();
	if (body == nullptr) {
		return;
	}

	// convert this anchor's global position to a global offset from the body's origin
	body->apply_force(p_force, get_global_position() - body->get_global_position());
}
