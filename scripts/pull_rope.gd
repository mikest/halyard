@tool
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

## Distance over length rope must stretch to before pulling back
@export var stretch_pull_threshold: float = 0.1

#region Runtime
func _ready() -> void:
	if Engine.is_editor_hint(): return
	
	# Bug: the transforms at the _end_ of the rope are messed up for LOD>1
	# so we force it here
	if appearance:
		appearance.rope_lod = 1


## This rope will pull on its end_anchor if the end anchor is a RigidBody3D
func _physics_process(_delta: float) -> void:
	var current := get_current_rope_length()
	var stretch := current - rope_length
	
	# the last link transform will be aligned along the rope Tangent, so we'll use that to se the force direction
	if (stretch > stretch_pull_threshold) and pull_on and end_anchor:
		
		var link_count := get_link_count()
		if link_count >= 4:
			# Bug: Use count - 2 link position as the last link xform is messed up from the catmul interpolation
			var end: Transform3D = global_transform * get_link(link_count-2)
		
			# use a node as the pull poisition.
			var pull_point := end.origin - pull_on.global_position
			var dir := -end.basis.y
		
			# cap the pull force at a multiple of the body mass to keep from wildly oscillating if the stretch is too large
			var force := pow(stretch, force_stiffness) * pull_on.mass * force_strength
			var force_limit := pull_on.mass * force_limit_mass_scale
			force = clampf(force, -force_limit, force_limit)

			var force_vector := dir * force
			pull_on.apply_force(force_vector, pull_point)
	pass
#endregion
