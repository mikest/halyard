@tool
extends EditorNode3DGizmoPlugin

func _get_gizmo_name():
	return "RopeFrames"

func _has_gizmo(node):
	return node is Rope

func _init():
	create_material("red", Color(1, 0, 0))
	create_material("green", Color(0, 1, 0))
	create_material("blue", Color(0, 0, 1))
	create_handle_material("handles")


func _redraw(gizmo):
	gizmo.clear()

	var rope: Rope = gizmo.get_node_3d()
	if rope:
		var red = PackedVector3Array()
		var green = PackedVector3Array()
		var blue = PackedVector3Array()

		var frames := rope.get_all_rope_frames()
		var xform := rope.global_transform.inverse()
		var d:= rope.get_rope_width()
		for idx in frames.size():
			var origin := frames[idx].origin
			var orientation := Transform3D(frames[idx].basis, Vector3.ZERO)
			red.push_back(origin)
			red.push_back(origin + (d * Vector3(1,0,0)) * orientation )
			green.push_back(origin)
			green.push_back(origin + d * Vector3(0,1,0) * orientation)
			blue.push_back(origin)
			blue.push_back(origin + d * Vector3(0,0,1) * orientation)
			
		gizmo.add_lines(red, get_material("red", gizmo), false)
		gizmo.add_lines(green, get_material("green", gizmo), false)
		gizmo.add_lines(blue, get_material("blue", gizmo), false)

		# var handles = PackedVector3Array()

		# handles.push_back(Vector3(0, 1, 0))
		# handles.push_back(Vector3(0, node3d.my_custom_value, 0))

		# gizmo.add_handles(handles, get_material("handles", gizmo), [])
