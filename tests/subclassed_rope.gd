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
func _get_anchor_count() -> int:
	return 1

func _get_anchor_position(idx: int) -> float:
	var pos := global_position + offset
	var dist := (pos - global_position).length() + (idx* 0.1)
	return dist / rope_length * (1.0 - anchor_tension)

func _get_anchor_transform(idx: int) -> Transform3D:
	return global_transform.translated(offset)


# Use all the child nodes as attachments and distribute them evenly along the rope
func _get_attachment_count() -> int:
	return get_child_count()

func _get_attachment_position(idx: int) -> float:
	var total : float = get_child_count() - 1;
	var span := 3
	var dist := (idx / total) * span;
	return 1.0 - (dist / rope_length);

func _get_attachment_nodepath(idx: int) -> NodePath:
	return get_path_to(get_child(idx))
	
func _get_attachment_transform(idx: int) -> Transform3D:
	var xform: Transform3D
	return xform.rotated_local(Vector3.UP, PI/3 * idx)

#endregion
