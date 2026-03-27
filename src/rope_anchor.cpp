/* Copyright (c) M. Estee. MIT License. */

#include "rope_anchor.h"

void RopeAnchor::_bind_methods() {
	ADD_SIGNAL(MethodInfo("anchor_changed"));

	EXPORT_PROPERTY_ENUM(behavior, "Free:-1,Anchored,Towing,Guided,Sliding", RopeAnchor);
	ClassDB::bind_method(D_METHOD("set_friction", "friction"), &RopeAnchor::set_friction);
	ClassDB::bind_method(D_METHOD("get_friction"), &RopeAnchor::get_friction);
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
	return Object::cast_to<RigidBody3D>(get_parent());
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
	notify_property_list_changed();
	emit_signal("anchor_changed");
}

void RopeAnchor::_get_property_list(List<PropertyInfo> *p_list) const {
	ERR_FAIL_NULL(p_list);

	// friction is only meaningful when the rope slides through this anchor
	const bool show = (_behavior == GUIDED || _behavior == TOWING);
	const uint32_t usage = show ? PROPERTY_USAGE_DEFAULT : PROPERTY_USAGE_NONE;
	p_list->push_back(PropertyInfo(Variant::FLOAT, "friction", PROPERTY_HINT_RANGE, "0.0,1.0,0.01", usage));
}

bool RopeAnchor::_set(const StringName &p_name, const Variant &p_property) {
	if (p_name == StringName("friction")) {
		set_friction((float)p_property);
		return true;
	}
	return false;
}

bool RopeAnchor::_get(const StringName &p_name, Variant &r_property) const {
	if (p_name == StringName("friction")) {
		r_property = _friction;
		return true;
	}
	return false;
}

void RopeAnchor::apply_force(const Vector3 &p_force) {
	RigidBody3D *body = get_parent_rigid_body();
	if (body == nullptr) {
		return;
	}

	// convert this anchor's global position to a global offset from the body's origin
	body->apply_force(p_force, get_global_position() - body->get_global_position());
}
