@tool
extends RopePositions
class_name DistributedRopePositions

@export var include_start: bool
@export var include_end: bool

func get_position_at_index(idx: int, _rope_length: float) -> float:
	var count := position_count
	if count:
		if include_start:
			count += 1
			idx += 1
		if include_end:
			#count += 1
			pass
		else:
			count -= 1
			
		return 1.0/(count) * idx
		
	return 0.0
