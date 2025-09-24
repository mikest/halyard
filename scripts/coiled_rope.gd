@tool
extends PullRope
class_name CoiledRope

@export var clockwise: bool = false:
	set(val):
		clockwise = val
		_dirty = true
@export_range(0.1,10,0.01) var radius :float = 0.125:
	set(val):
		radius = val
		_dirty = true
@export var turns_per_layer := 2.0:
	set(val):
		turns_per_layer = val
		_dirty = true

@export var coil_to_first_anchor_tension := 0.8
@export var offset_turns := 0.0
@export var debug: bool = false

## What ratio of the rope is coiled.
@export_range(0,1,0.001) var coiled := 0.5

# global coordinates of the points along the coil for the entire rope length
var _positions: PackedVector3Array

# accumulated turn amount for each point along the coil
var _turns: PackedFloat32Array

# positions and turn need to be rebuilt
var _dirty: bool = true


func _process(delta: float) -> void:
	if debug:
		DebugDraw3D.draw_point_path(_positions, DebugDraw3D.POINT_TYPE_SQUARE, 0.01, Color.BLUE)
	
	# rebuild positions for coils when the parameters change
	if _dirty:
		_rebuild_coil_positions()
		_dirty = false


func _get_anchors_per_turn(r: float) -> int:
	var circumfrence := TAU * r
	return circumfrence * get_particles_per_meter()


func _get_coil_anchor_count() -> int:
	var coil_length := rope_length * coiled
	return int(coil_length * get_particles_per_meter()) 


func _rebuild_coil_positions():
	_positions = PackedVector3Array()
	_turns = PackedFloat32Array()
	var count := int(rope_length * get_particles_per_meter())
	
	# accumulated distance
	var turn := 0.0
	for idx in count:
		var offset_per_layer := get_rope_width() * 0.25
		var offset_per_turn := get_rope_width() * 0.6
		
		# how many anchors in a turn
		var layer : int = floor(turn / turns_per_layer)
		var layer_radius := radius + offset_per_layer * layer
		var layer_circumfrence := TAU * layer_radius
		
		var anchors_per_turn := layer_circumfrence * get_particles_per_meter()

		var turn_dir := -1.0 if int(layer) % 2==0 else 1.0
		
		var layer_turn := wrapf(turn, 0, turns_per_layer)
		var turn_x_offset := 0 if turn_dir<0 else (-offset_per_turn * turns_per_layer)
		var x := layer_turn * offset_per_turn * turn_dir + float(turn_x_offset)
		var y := layer_radius + offset_per_layer * layer
		
		#offset_turns = wrapf(coiled * rope_length, 0, layer_radius*TAU)
		
		var axis := Vector3(1,0,0)
		var xform: Transform3D
		xform = xform.translated_local(Vector3(x, y, 0))
		xform = xform.rotated(axis, turn * TAU * (-1 if clockwise else 1))
		
		_positions.push_back(xform.origin)
		_turns.push_back(turn)
		
		# advance accumulators
		turn += 1.0 / anchors_per_turn


func _get_nodepath_transform(path: NodePath) -> Transform3D:
	var xform: Transform3D = global_transform
	var node: Node3D = get_node_or_null(path)
	if node:
		xform = node.global_transform
	return xform

# get the transform for the regular anchor
func _get_transform_for_anchor_idx(anchor_idx) -> Transform3D:
	var path := anchors.get_node(anchor_idx)
	return _get_nodepath_transform(path)


#region Overloads
func _get_anchor_count() -> int:
	# specific anchors come after the coil
	var count := _get_coil_anchor_count()
	if anchors:
		count += anchors.position_count
	
	return count


func _get_anchor_position(idx: int) -> float:
	var count := _get_anchor_count()
	var coil_count := _get_coil_anchor_count()
	var pos := -1.0
	
	if idx >= coil_count:
		var coil_end := _get_anchor_transform(idx-1)
		var anchor := _get_anchor_transform(idx)
		var dist := coil_end.origin.distance_to(anchor.origin)
		pos = coiled + (dist/rope_length) * coil_to_first_anchor_tension
	else:
		pos = idx / float(count) * coiled
	
	return pos


func _get_anchor_transform(idx: int) -> Transform3D:
	var xform := global_transform
	var count := _get_anchor_count()
	var coil_count := _get_coil_anchor_count()
	
	var start_xform := _get_nodepath_transform(start_anchor)
	
	# if we're past the coil return the normal anchors
	if idx >= coil_count:
		var path := anchors.get_node(idx - coil_count)
		xform = _get_nodepath_transform(path)
	
	# return the rotated coil position for this index
	elif idx < _positions.size():
		xform = Transform3D(Basis(), _positions[idx])
		xform = xform.rotated(Vector3(1,0,0), (-_turns[coil_count] + offset_turns) * TAU * (-1 if clockwise else 1))
		xform = start_xform * xform
	return xform
#endregion
