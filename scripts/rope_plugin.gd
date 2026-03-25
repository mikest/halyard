@tool
extends EditorPlugin

# const DistributedAnchors = preload("scripts/distributed_anchors.gd")
# const DistributedAttachments = preload("scripts/distributed_attachments.gd")

const RopeParticlesGizmo = preload("rope_particles_gizmo.gd")
const RopePositionsGizmo = preload("rope_positions_gizmo.gd")
const RopeFramesGizmo = preload("rope_frames_gizmo.gd")
const RopeInspectorPlugin = preload("rope_inspector_plugin.gd")
const RopeMenu = preload("rope_menu.gd")

const RopeIcon = preload("../icons/Rope.svg")
const RopeAppearanceIcon = preload("../icons/RopeAppearance.svg")
const RopePositionsIcon = preload("../icons/RopePositions.svg")

var rope_particles_gizmo := RopeParticlesGizmo.new()
var rope_frames_gizmo := RopeFramesGizmo.new()
var rope_positions_gizmo := RopePositionsGizmo.new()
var rope_inspector_plugin := RopeInspectorPlugin.new()
var rope_menu: RopeMenu = RopeMenu.new()

# currently selected rope
var current_rope: Rope = null

func _enable_plugin() -> void:
	pass


func _disable_plugin() -> void:
	pass


func _handles(p_object: Object) -> bool:
	if p_object is Rope:
		return true
	return false


func _make_visible(p_visible: bool, p_redraw: bool = false) -> void:
	if current_rope:
		rope_menu.visible = true
	else:
		rope_menu.visible = false


func _edit(object: Object) -> void:
	if not object:
		_clear()
	
	if object is Rope:
		current_rope = object
		rope_menu.visible = true


func _clear() -> void:
	current_rope = null
	rope_menu.visible = false


func _enter_tree() -> void:
	# Initialization of the plugin goes here.
	add_node_3d_gizmo_plugin(rope_particles_gizmo)
	add_node_3d_gizmo_plugin(rope_positions_gizmo)
	add_node_3d_gizmo_plugin(rope_frames_gizmo)
	
	add_inspector_plugin(rope_inspector_plugin)
	add_control_to_container(EditorPlugin.CONTAINER_SPATIAL_EDITOR_MENU, rope_menu)
	rope_menu.plugin = self
	pass


func _exit_tree() -> void:
	# Clean-up of the plugin goes here.
	remove_node_3d_gizmo_plugin(rope_particles_gizmo)
	remove_node_3d_gizmo_plugin(rope_positions_gizmo)
	remove_node_3d_gizmo_plugin(rope_frames_gizmo)
	
	remove_inspector_plugin(rope_inspector_plugin)
	remove_control_from_container(EditorPlugin.CONTAINER_SPATIAL_EDITOR_MENU, rope_menu)
	rope_menu.plugin = null
	pass
