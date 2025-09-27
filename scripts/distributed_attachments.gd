@tool
extends RopeAttachmentsBase
class_name DistributedAttachments

@export var include_start: bool
@export var include_end: bool

func get_count(rope: Rope) -> int:
	return rope.get_child_count()

func get_position(idx: int, rope: Rope) -> float:
	var count := get_count(rope)
	if count:
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

func get_nodepath(idx: int, rope: Rope) -> NodePath:
	var node := rope.get_child(idx)
	if node:
		return rope.get_path_to(node)
	return ""
