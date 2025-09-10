@tool
extends EditorNode3DGizmoPlugin

func _get_gizmo_name():
	return "RopeParticles"

func _has_gizmo(node):
	return node is Rope

func _init():
	create_material("main", Color(1, 1, 0))


func _redraw(gizmo):
	gizmo.clear()

	var rope: Rope = gizmo.get_node_3d()
	if rope:
		var lines = PackedVector3Array()
		
		var positions := rope.get_particle_positions()
		var xform := rope.global_transform.inverse()
		
		var orientation := Transform3D(xform.basis, Vector3.ZERO)
		orientation = orientation.rotated_local(Vector3.RIGHT, PI/4)
		orientation = orientation.rotated_local(Vector3.UP, PI/4)
		
		var d:= rope.get_rope_width()
		for idx in positions.size():
			lines.push_back(xform * positions[idx])
			lines.push_back(xform * positions[idx] + d * Vector3.UP * orientation)
			lines.push_back(xform * positions[idx])
			lines.push_back(xform * positions[idx] + d * Vector3.DOWN * orientation)
			lines.push_back(xform * positions[idx])
			lines.push_back(xform * positions[idx] + d * Vector3.LEFT * orientation)
			lines.push_back(xform * positions[idx])
			lines.push_back(xform * positions[idx] + d * Vector3.RIGHT * orientation)
			lines.push_back(xform * positions[idx])
			lines.push_back(xform * positions[idx] + d * Vector3.MODEL_FRONT * orientation)
			lines.push_back(xform * positions[idx])
			lines.push_back(xform * positions[idx] + d * Vector3.MODEL_REAR * orientation)
			
		gizmo.add_lines(lines, get_material("main", gizmo), false)
