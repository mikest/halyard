@tool
extends RopePositions
class_name OffsetRopePositions

func get_position_at_index(idx: int, rope_length: float) -> float:	
	var pos := get_position(idx)
	return 1.0 - pos/rope_length
