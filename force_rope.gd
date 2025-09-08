@tool
@icon("res://icons/ring-icon.png")
extends Rope
class_name ForceRope

@export var strength := 1.0

#region Runtime
func _ready() -> void:
	if Engine.is_editor_hint(): return
	pass

## This rope will pull on its end_anchor if the end anchor is a RigidBody3D
func _physics_process(_delta: float) -> void:
	var stretch := get_current_rope_length() - rope_length
	var anchor: RigidBody3D = get_node_or_null(end_anchor)
	if appearance:
		# the attachment will be aligned along the rope Tangent, so we'll use that to se the force direction
		var attachment: Node3D = get_node_or_null(appearance.end_attachment)
		if stretch > 0.01 and anchor and attachment:
			var xform := attachment.global_transform
			
			# pull on the origin, not the center of mass so that we can have objects swinging from a joint.
			anchor.apply_force(-xform.basis.y.normalized() * stretch * anchor.mass * strength, Vector3(0,0,0))
	pass
#endregion
