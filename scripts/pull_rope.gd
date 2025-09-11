@tool
@icon("res://icons/ring-icon.png")
extends Rope
class_name PullRope

## This node pulls on a RigidBody3D and applies the force at the [code]end_anchor[/code]
## relative to the RigidBody3D's origin. This means that you probably want the end anchor
## to be a child of the RigidBody.

## Objcet to pull on.
## The object force will be applied at end_anchor
@export var pull_on: RigidBody3D

## How hard to pull as a scalar of mass
@export var force_strength := 1.0

## How stiff the rope is, lower values are stretchier. Higher valyues
## may introduce oscillations.
@export var force_stiffness := 5.0

## Multiplied by pullo_on.mass and used to cap the upper limit for force.
## useful for preventing oscillations.
@export var force_limit_mass_scale := 100.0

#region Runtime
func _ready() -> void:
	if Engine.is_editor_hint(): return
	pass

## This rope will pull on its end_anchor if the end anchor is a RigidBody3D
func _physics_process(_delta: float) -> void:
	var current := get_current_rope_length()
	var stretch := current - rope_length
	
	# the attachment will be aligned along the rope Tangent, so we'll use that to se the force direction
	if pull_on and end_anchor:
		var count := get_rope_frame_count()
		var xform : Transform3D = get_rope_frame(count-1)
		xform = global_transform * xform
		#DebugDraw3D.draw_gizmo(xform)
		
		# use a node as the pull poisition.
		var pull_point := Vector3(0,0,0)
		var pull_node :Node3D = get_node_or_null(end_anchor)
		if pull_node:
			pull_point = pull_node.global_position - pull_on.global_position
		
		# cap the pull force at a multiple of the body mass to keep from wildly oscillating if the stretch is too large
		var force := pow(stretch,force_stiffness) * pull_on.mass * force_strength
		var force_limit := pull_on.mass * force_limit_mass_scale
		force = clampf(force, -force_limit, force_limit)
		
		# pull on the origin, not the center of mass so that we can have objects swinging from a joint.
		var force_vector := -xform.basis.y.normalized() * force
		pull_on.apply_force(force_vector, pull_point)
	pass
#endregion
