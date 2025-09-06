#include "rope_appearance.h"

void RopeAppearance::_bind_methods() {
	// rope parameters
	EXPORT_PROPERTY(Variant::FLOAT, rope_width);
	EXPORT_PROPERTY(Variant::INT, rope_sides);
	EXPORT_PROPERTY(Variant::FLOAT, rope_twist);
	EXPORT_PROPERTY(Variant::INT, rope_lod);

	ClassDB::bind_method(D_METHOD("set_material", "material"), &set_material);
	ClassDB::bind_method(D_METHOD("get_material"), &get_material);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "material", PROPERTY_HINT_RESOURCE_TYPE, "Material"), "set_material", "get_material");

	// attachments
	EXPORT_PROPERTY(Variant::NODE_PATH, start_attachment);
	ClassDB::bind_method(D_METHOD("set_attachments", "attachments"), &set_attachments);
	ClassDB::bind_method(D_METHOD("get_attachments"), &get_attachments);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "attachments", PROPERTY_HINT_RESOURCE_TYPE, "RopePositions"), "set_attachments", "get_attachments");
	EXPORT_PROPERTY(Variant::NODE_PATH, end_attachment);

	// simulation parameters
	ADD_GROUP("Simulation", "");
	EXPORT_PROPERTY(Variant::INT, particles_per_meter);
}

void RopeAppearance::set_material(const Ref<Material> &p_material) {
	_material = p_material;
	_notify_change();
}

Ref<Material> RopeAppearance::get_material() const {
	return _material;
}

#pragma endregion