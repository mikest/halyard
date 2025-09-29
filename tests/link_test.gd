@tool
extends Rope

@export_range(0,10,.01) var grow_length := 5.0
@export_range(0,10,.01) var grow_speed := 2.0

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
var phase := 0.0
func _process(delta: float) -> void:
	
	phase = wrapf(phase + delta * grow_speed, -PI, PI)
	rope_length = grow_length + sin(phase) * grow_length
	
	var label: Label3D = get_node_or_null("%Label3D")
	if label:
		var text = "stretch: %f\n" % [get_current_rope_length() - rope_length]
		# print out an ellided list of points
		var max := 3
		var points := get_particle_positions()
		var idx := 0
		var once:= true
		for p in points:
			if (idx < max) or (idx > points.size()-(max+1)):
				text += "%d %s\n" % [idx, str(p)]
			elif once:
				text += "...\n"
				once = false
			idx += 1
		label.text = text
	pass
