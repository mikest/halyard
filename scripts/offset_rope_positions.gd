@tool
extends RopePositions
class_name OffsetRopePositions

enum From {
	Start,
	End
}

@export var from: From = From.Start

func get_position_at_index(idx: int, rope_length: float) -> float:	
	var pos := get_position(idx)
	if from == From.Start:
		return pos/rope_length
	else:
		return 1.0 - pos/rope_length
