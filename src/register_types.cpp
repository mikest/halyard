#include "register_types.h"

#include <gdextension_interface.h>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

#include "rope.h"
#include "rope_anchor_positions.h"
#include "rope_anchors_base.h"
#include "rope_appearance.h"
#include "rope_attachment_positions.h"
#include "rope_attachments_base.h"
#include "buoyancy.h"
#include "character_buoyancy.h"
#include "liquid_area.h"

using namespace godot;

void initialize_gdextension_types(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
	GDREGISTER_CLASS(Rope);

	GDREGISTER_VIRTUAL_CLASS(RopeAnchorsBase);
	GDREGISTER_VIRTUAL_CLASS(RopeAttachmentsBase);

	GDREGISTER_CLASS(RopeAnchorPositions);
	GDREGISTER_CLASS(RopeAttachmentPositions);
	GDREGISTER_CLASS(RopeAppearance);
	GDREGISTER_CLASS(CharacterBuoyancy);
	GDREGISTER_CLASS(Buoyancy);
	GDREGISTER_CLASS(LiquidArea);
}

void uninitialize_gdextension_types(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
}

extern "C" {
// Initialization
GDExtensionBool GDE_EXPORT halyard_library_init(
		GDExtensionInterfaceGetProcAddress p_get_proc_address,
		GDExtensionClassLibraryPtr p_library,
		GDExtensionInitialization *r_initialization) {
	GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);
	init_obj.register_initializer(initialize_gdextension_types);
	init_obj.register_terminator(uninitialize_gdextension_types);
	init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);

	return init_obj.init();
}
}