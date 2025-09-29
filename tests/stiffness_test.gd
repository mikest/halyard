@tool
extends Node3D


# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
var phase := 0.0
func _process(delta: float) -> void:
	phase = wrapf(phase + delta * 10.0, -PI, PI)
	global_position.y = sin(phase) * 0.25
	pass
