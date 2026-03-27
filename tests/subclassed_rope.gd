@tool
extends Rope

## A subclassed rope that demonstrates some ways in which the
## behavior of the anchors and attachments can be derrived in code
## instead of as a resource

## how tight the rope is between the start and the single anchor
@export var anchor_tension := 0.5

@export var offset:= Vector3(3,1,0)

#region Runtime

var _phase := 0.0
func _process(delta: float):
	_phase = wrapf(_phase + delta, 0.0, 2*PI)
	rope_length = 3 + 3 * (sin(_phase) + 1)/2.0

# Create a single anchor point that is always a fixed distance from the start
func _update_anchors() -> void:
	anchor_count = 2
	for idx in anchor_count:
		set_anchor_offset(idx, _get_anchor_abs_offset(idx))
		set_anchor_transform(idx, _get_anchor_transform(idx))
		set_anchor_behavior(idx, RopeAnchor.AnchorBehavior.ANCHORED)
		set_anchor_nodepath(idx, "")


func _get_anchor_abs_offset(idx: int) -> float:
	if idx == 0:
		return 0
	
	var pos := global_position + offset
	var dist := (pos - global_position).length() + (idx* 0.1)
	return dist * (1.0 - anchor_tension)


func _get_anchor_transform(idx: int) -> Transform3D:
	if idx == 0:
		return global_transform
		
	return global_transform.translated(offset)



# Use all the child nodes as attachments and distribute them evenly along the rope
func _update_attachments() -> void:
	if appearance:
		appearance.attachment_count = get_child_count()
		
		var last := appearance.attachment_count - 1;
		for idx in appearance.attachment_count:
			var span := 3
			var dist := (idx / last) * span;
			appearance.set_attachment_offset(idx, rope_length - dist)
			
			var path := get_path_to(get_child(last - idx))
			appearance.set_attachment_nodepath(idx, path)

func _get_attachment_local_transform(idx: int) -> Transform3D:
	var xform: Transform3D
	if appearance:
		if idx == appearance.attachment_count - 1:
			xform = xform.rotated_local(Vector3.RIGHT, PI)
		else:
			xform = xform.rotated_local(Vector3.UP, PI/3 * idx)
		
	return xform


#endregion
