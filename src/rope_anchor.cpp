/* Copyright (c) M. Estee. MIT License. */

#include "rope_anchor.h"

void RopeAnchor::_bind_methods() {
	ADD_SIGNAL(MethodInfo("anchor_changed"));

	EXPORT_PROPERTY_ENUM(behavior, "Free:-1,Anchored,Towing,Guided,Sliding", RopeAnchor);
	BIND_ENUM_CONSTANT(FREE);
	BIND_ENUM_CONSTANT(ANCHORED); // Rope can't move anchor.
	BIND_ENUM_CONSTANT(TOWING); // Rope pulls on rigid body at this point
	BIND_ENUM_CONSTANT(GUIDED); // Rope travels freely through this point, but rope cannot move it.
	BIND_ENUM_CONSTANT(SLIDING);

	ClassDB::bind_method(D_METHOD("get_parent_rigid_body"), &RopeAnchor::get_parent_rigid_body);
	ClassDB::bind_method(D_METHOD("apply_force", "force"), &RopeAnchor::apply_force);

	ClassDB::bind_method(D_METHOD("get_particle_count"), &RopeAnchor::get_particle_count);
	ClassDB::bind_method(D_METHOD("get_particle_position", "idx"), &RopeAnchor::get_particle_position);

	GDVIRTUAL_BIND(get_particle_count);
	GDVIRTUAL_BIND(get_particle_position, "idx");
}

RigidBody3D *RopeAnchor::get_parent_rigid_body() const {
	Node *parent = get_parent();
	if (parent == nullptr) {
		return nullptr;
	}
	return Object::cast_to<RigidBody3D>(parent);
}

int RopeAnchor::get_particle_count() const {
	// allow subclasses to override the count via GDScript/GDExtension
	if (GDVIRTUAL_IS_OVERRIDDEN(get_particle_count)) {
		int count = 1;
		GDVIRTUAL_CALL(get_particle_count, count);
		return MAX(1, count);
	}
	// base anchor always provides exactly one position
	return 1;
}

Vector3 RopeAnchor::get_particle_position(int p_idx) const {
	// allow subclasses to override the position via GDScript/GDExtension
	if (GDVIRTUAL_IS_OVERRIDDEN(get_particle_position)) {
		Vector3 pos;
		GDVIRTUAL_CALL(get_particle_position, p_idx, pos);
		return pos;
	}
	// default: the anchor sits at this node's local origin
	return Vector3();
}

void RopeAnchor::_notify_anchor_changed() {
	emit_signal("anchor_changed");
}

void RopeAnchor::apply_force(const Vector3 &p_force) {
	RigidBody3D *body = get_parent_rigid_body();
	if (body == nullptr) {
		return;
	}

	// convert this anchor's global position to a global offset from the body's origin
	body->apply_force(p_force, get_global_position() - body->get_global_position());
}


