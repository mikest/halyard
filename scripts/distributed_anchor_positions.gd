@tool
extends RopeAnchorPositions
class_name DistributedAnchorPositions

@export var include_start: bool
@export var include_end: bool

# func get_count(rope: Rope) -> int:
	# return get_particle_positions()

func get_position(idx: int, rope: Rope) -> float:
	var count := get_count(rope)
	if count > 0:
		if include_start:
			count += 1
			idx += 1
		if include_end:
			#count += 1
			pass
		else:
			count -= 1
			
		return 1.0 / (count) * idx
		
	return 0.0
