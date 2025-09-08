@tool
@icon("res://icons/ring-icon.png")
extends Rope
class_name PullRope

## To use this rope to pull on a rigid body end_anchor the rope
## to either pull_on or pull_from and then anchor the start_anchor
## to your fixed point.

## Objcet to pull on.
## Probably want to set anchor for the rope to either this object or pull_from
@export var pull_on: RigidBody3D

## Use a different node to set the point to pull on
## for the RigidBody. Otherwise the RigidBody origin will be used.
@export var pull_from: NodePath

## How hard to pull as a scalar of mass
@export var force_strength := 1.0

## How stiff the rope is, lower values are stretchier
@export var force_stiffness := 5.0

## Multiplied by pullo_on.mass and used to cap the upper limit for force.
## useful for preventing oscillations.
@export var force_limit_mass_scale := 10.0

#region Runtime
func _ready() -> void:
	if Engine.is_editor_hint(): return
	pass

## This rope will pull on its end_anchor if the end anchor is a RigidBody3D
func _physics_process(_delta: float) -> void:
	var stretch := get_current_rope_length() - rope_length
	
	# the attachment will be aligned along the rope Tangent, so we'll use that to se the force direction
	var xform := get_rope_frame_at_offset(1.0)
	if pull_on:
		var pull_point := Vector3(0,0,0)
		
		# use a node as the pull poisition.
		var pull_node :Node3D = get_node_or_null(pull_from)
		if pull_node:
			pull_point = pull_node.global_position - pull_on.global_position
		
		# pull on the origin, not the center of mass so that we can have objects swinging from a joint.
		var force := pow(stretch,force_stiffness)
		
		# cap the pull force at a multiple of the body mass to keep from wildly oscillating if the stretch is too large
		var force_limit := pull_on.mass * force_limit_mass_scale
		force = clampf(force, -force_limit, force_limit)
		
		pull_on.apply_force(-xform.basis.y.normalized() * \
		# stiffness is done as a power function so that the rope pulls harder back to the end the further its been streached
			force * \
			pull_on.mass * force_strength, pull_point)
	pass
#endregion
