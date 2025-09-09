@tool
extends EditorPlugin

const PullRope = preload("scripts/pull_rope.gd")
const DistributedRopePositions = preload("scripts/distributed_rope_positions.gd")
const OffsetRopePositions = preload("scripts/offset_rope_positions.gd")


const RopeIcon = preload("icons/Rope.svg")
const RopeAppearanceIcon = preload("icons/RopeAppearance.svg")
const RopePositionsIcon = preload("icons/RopePositions.svg")


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
    pass


func _exit_tree() -> void:
    # Clean-up of the plugin goes here.
    pass