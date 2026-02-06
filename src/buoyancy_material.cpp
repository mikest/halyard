/* BuoyancyMaterial
 *
 * A Resource that holds buoyancy-related physics properties for a body.
 *
 * Copyright (c) M. Estee
 * MIT License.
 */

#include "buoyancy_material.h"

using namespace godot;

#pragma region Properties

void BuoyancyMaterial::set_buoyancy(float p_buoyancy) {
	_buoyancy = p_buoyancy;
	emit_changed();
}

float BuoyancyMaterial::get_buoyancy() const {
	return _buoyancy;
}

void BuoyancyMaterial::set_linear_drag(float p_drag) {
	_linear_drag = p_drag;
	emit_changed();
}

float BuoyancyMaterial::get_linear_drag() const {
	return _linear_drag;
}

void BuoyancyMaterial::set_angular_drag(float p_drag) {
	_angular_drag = p_drag;
	emit_changed();
}

float BuoyancyMaterial::get_angular_drag() const {
	return _angular_drag;
}

void BuoyancyMaterial::set_linear_drag_scale(const Vector3 &p_scale) {
	_linear_drag_scale = p_scale;
	emit_changed();
}

Vector3 BuoyancyMaterial::get_linear_drag_scale() const {
	return _linear_drag_scale;
}

void BuoyancyMaterial::set_angular_drag_scale(const Vector3 &p_scale) {
	_angular_drag_scale = p_scale;
	emit_changed();
}

Vector3 BuoyancyMaterial::get_angular_drag_scale() const {
	return _angular_drag_scale;
}

#pragma endregion

#pragma region Bindings

void BuoyancyMaterial::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_buoyancy", "buoyancy"), &BuoyancyMaterial::set_buoyancy);
	ClassDB::bind_method(D_METHOD("get_buoyancy"), &BuoyancyMaterial::get_buoyancy);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "buoyancy", PROPERTY_HINT_RANGE, "0,100,0.1"), "set_buoyancy", "get_buoyancy");

	ClassDB::bind_method(D_METHOD("set_linear_drag", "linear_drag"), &BuoyancyMaterial::set_linear_drag);
	ClassDB::bind_method(D_METHOD("get_linear_drag"), &BuoyancyMaterial::get_linear_drag);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "linear_drag", PROPERTY_HINT_RANGE, "0,100,0.01"), "set_linear_drag", "get_linear_drag");

	ClassDB::bind_method(D_METHOD("set_angular_drag", "angular_drag"), &BuoyancyMaterial::set_angular_drag);
	ClassDB::bind_method(D_METHOD("get_angular_drag"), &BuoyancyMaterial::get_angular_drag);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "angular_drag", PROPERTY_HINT_RANGE, "0,100,0.01"), "set_angular_drag", "get_angular_drag");

	ClassDB::bind_method(D_METHOD("set_linear_drag_scale", "linear_drag_scale"), &BuoyancyMaterial::set_linear_drag_scale);
	ClassDB::bind_method(D_METHOD("get_linear_drag_scale"), &BuoyancyMaterial::get_linear_drag_scale);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "linear_drag_scale"), "set_linear_drag_scale", "get_linear_drag_scale");

	ClassDB::bind_method(D_METHOD("set_angular_drag_scale", "angular_drag_scale"), &BuoyancyMaterial::set_angular_drag_scale);
	ClassDB::bind_method(D_METHOD("get_angular_drag_scale"), &BuoyancyMaterial::get_angular_drag_scale);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "angular_drag_scale"), "set_angular_drag_scale", "get_angular_drag_scale");
}

#pragma endregion
