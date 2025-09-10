@tool
extends EditorPlugin

const PullRope = preload("scripts/pull_rope.gd")
const DistributedRopePositions = preload("scripts/distributed_rope_positions.gd")
const OffsetRopePositions = preload("scripts/offset_rope_positions.gd")

const RopeParticlesGizmo = preload("scripts/rope_particles_gizmo.gd")
const RopePositionsGizmo = preload("scripts/rope_positions_gizmo.gd")
const RopeFramesGizmo = preload("scripts/rope_frames_gizmo.gd")


const RopeIcon = preload("icons/Rope.svg")
const RopeAppearanceIcon = preload("icons/RopeAppearance.svg")
const RopePositionsIcon = preload("icons/RopePositions.svg")

var rope_particles_gizmo = RopeParticlesGizmo.new()
var rope_frames_gizmo = RopeFramesGizmo.new()
var rope_positions_gizmo = RopePositionsGizmo.new()

func _enable_plugin() -> void:
	add_custom_type("PullRope", "Rope", PullRope, RopeIcon)
	

	add_custom_type("DistributedRopePositions", "RopePositions", DistributedRopePositions, RopePositionsIcon)
	add_custom_type("OffsetRopePositions", "RopePositions", OffsetRopePositions, RopePositionsIcon)


func _disable_plugin() -> void:
	remove_custom_type("PullRope")

	remove_custom_type("DistributedRopePositions")
	remove_custom_type("OffsetRopePositions")
	pass


func _enter_tree() -> void:
	# Initialization of the plugin goes here.
	add_node_3d_gizmo_plugin(rope_particles_gizmo)
	add_node_3d_gizmo_plugin(rope_positions_gizmo)
	add_node_3d_gizmo_plugin(rope_frames_gizmo)
	pass


func _exit_tree() -> void:
	# Clean-up of the plugin goes here.
	remove_node_3d_gizmo_plugin(rope_particles_gizmo)
	remove_node_3d_gizmo_plugin(rope_positions_gizmo)
	remove_node_3d_gizmo_plugin(rope_frames_gizmo)
	pass
