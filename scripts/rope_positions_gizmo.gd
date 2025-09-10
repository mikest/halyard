@tool
extends EditorNode3DGizmoPlugin

const anchor_icon := preload("res://addons/halyard/icons/Anchor.svg")
const end_anchor_icon := preload("res://addons/halyard/icons/EndAnchor.svg")
const start_anchor_icon := preload("res://addons/halyard/icons/StartAnchor.svg")

const attach_icon := preload("res://addons/halyard/icons/Attachment.svg")
const end_attach_icon := preload("res://addons/halyard/icons/EndAttachment.svg")
const start_attach_icon := preload("res://addons/halyard/icons/StartAttachment.svg")

func _get_gizmo_name():
	return "RopePositions"

func _has_gizmo(node):
	return node is Rope

func _init():
	create_handle_material("anchor", false, anchor_icon)
	create_handle_material("start_anchor", false, start_anchor_icon)
	create_handle_material("end_anchor", false, end_anchor_icon)
	create_handle_material("attach", false, attach_icon)
	create_handle_material("start_attach", false, start_attach_icon)
	create_handle_material("end_attach", false, end_attach_icon)
	create_material("main", Color(1, 1, 0))


func _redraw(gizmo: EditorNode3DGizmo):
	gizmo.clear()

	var rope: Rope = gizmo.get_node_3d()
	if rope:
		var anchors := rope.anchors
		if anchors:
			_draw_positions(gizmo, rope, rope.start_anchor, rope.end_anchor, anchors, "anchor", false)
		
		var attach := rope.get_attachments()
		if attach:
			_draw_positions(gizmo, rope, rope.get_start_attachment(), rope.get_end_attachment(), attach, "attach", true)

func _draw_positions(gizmo:EditorNode3DGizmo, rope: Rope, start_path: NodePath, end_path: NodePath, positions: RopePositions, style: StringName, flip: bool):
	var lines = PackedVector3Array()
	var handles = PackedVector3Array()
	var start = PackedVector3Array()
	var end = PackedVector3Array()
	
	var count := positions.position_count
	for idx in count:
		var path := positions.get_node(idx)
		_push_positions_for_node(path, rope, lines, handles, flip)
	
	_push_positions_for_node(start_path, rope, lines, start, flip)
	_push_positions_for_node(end_path, rope, lines, end, flip)
	
	if lines.size():
		gizmo.add_lines(lines, get_material("main", gizmo))
	if handles.size():
		gizmo.add_handles(handles, get_material(style, gizmo), [])
	if start.size():
		gizmo.add_handles(start, get_material("start_" + style, gizmo), [])
	if end.size():
		gizmo.add_handles(end, get_material("end_" + style, gizmo), [])

func _push_positions_for_node(path: NodePath, rope:Rope, lines:PackedVector3Array, handles: PackedVector3Array, flip:bool):
		var node : Node3D = rope.get_node_or_null(path)
		if node:
			var xform := rope.global_transform.inverse()
			var orientation := Transform3D(xform.basis, Vector3.ZERO)
			orientation = orientation.rotated_local(Vector3.RIGHT, PI/4)
			orientation = orientation.rotated_local(Vector3.UP, PI/4)
			var d:= rope.get_rope_width() * 2
		
			var origin := node.global_position
			var dir := Vector3.DOWN if flip else Vector3.UP
			lines.push_back(xform * origin)
			lines.push_back(xform * origin + d * dir * orientation)
			
			var handle_pos := xform * origin + d * 1.5 * dir * orientation
			handles.push_back(handle_pos)
