#include "register_types.h"

#include <gdextension_interface.h>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

#include "buoyancy_material.h"
#include "character_buoyancy.h"
#include "clip_map.h"
#include "liquid_area.h"
#include "ocean_area.h"
#include "ocean_detailer.h"
#include "rigid_buoyancy.h"
#include "rope.h"
#include "rope_anchor_positions.h"
#include "rope_anchors_base.h"
#include "rope_appearance.h"
#include "rope_attachment_positions.h"
#include "rope_attachments_base.h"
#include "wave_sampler.h"

using namespace godot;

void initialize_gdextension_types(ModuleInitializationLevel p_level) {
	switch (p_level) {
		case MODULE_INITIALIZATION_LEVEL_CORE: {
		} break;

		case MODULE_INITIALIZATION_LEVEL_SERVERS: {
		} break;

		case MODULE_INITIALIZATION_LEVEL_SCENE: {
			ClassDB::register_class<Rope>();
			ClassDB::register_class<RopeAnchorsBase>();
			ClassDB::register_class<RopeAttachmentsBase>();
			ClassDB::register_class<RopeAnchorPositions>();
			ClassDB::register_class<RopeAttachmentPositions>();
			ClassDB::register_class<RopeAppearance>();
			ClassDB::register_class<CharacterBuoyancy>();
			ClassDB::register_class<RigidBuoyancy>();
			ClassDB::register_class<BuoyancyMaterial>();
			ClassDB::register_class<LiquidArea>();
			ClassDB::register_class<WaveSampler>();
			ClassDB::register_class<ClipMap>();
			ClassDB::register_class<ClipMapInstance>();
			ClassDB::register_class<OceanArea>();
			ClassDB::register_class<OceanDetailer>();
			break;
		}

		case MODULE_INITIALIZATION_LEVEL_EDITOR: {
			// OceanDetailer::_global_shader_init();
		} break;
		default:
			break;
	}
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
	init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SERVERS);

	return init_obj.init();
}
}