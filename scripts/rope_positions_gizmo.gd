@tool
extends EditorNode3DGizmoPlugin

const anchor_icon := preload("res://addons/halyard/icons/Anchor.svg")
const end_anchor_icon := preload("res://addons/halyard/icons/EndAnchor.svg")
const start_anchor_icon := preload("res://addons/halyard/icons/StartAnchor.svg")

const attach_icon := preload("res://addons/halyard/icons/Attachment.svg")
const end_attach_icon := preload("res://addons/halyard/icons/EndAttachment.svg")
const start_attach_icon := preload("res://addons/halyard/icons/StartAttachment.svg")

const guided_icon := preload("res://addons/halyard/icons/Guided.svg")
const sliding_icon := preload("res://addons/halyard/icons/Sliding.svg")
const towing_icon := preload("res://addons/halyard/icons/Towing.svg")

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
	create_material("start", Color(1, 1, 1))
	create_material("end", Color(.5, .5, .5))

	create_handle_material("guided", false, guided_icon)
	create_handle_material("sliding", false, sliding_icon)
	create_handle_material("towing", false, towing_icon)


func _redraw(gizmo: EditorNode3DGizmo):
	gizmo.clear()

	var rope: Rope = gizmo.get_node_3d()
	if rope:
		_draw_anchors(gizmo, rope, "anchor", false)
		
		var attach := rope.get_attachments()
		_draw_attachments(gizmo, rope, rope.get_start_attachment(), rope.get_end_attachment(), attach, "attach", true)

func _draw_anchors(gizmo: EditorNode3DGizmo, rope: Rope, style: StringName, flip: bool):
	var lines = PackedVector3Array()
	var handles = PackedVector3Array()
	var start = PackedVector3Array()
	var end = PackedVector3Array()
	
	var count := rope.get_anchor_count()
	for idx in count:
		var xform := rope.get_anchor_transform(idx)
		if idx == 0:
			_push_position(xform, rope, lines, start, flip, 0)
		elif idx == count-1:
			_push_position(xform, rope, lines, end, flip, count - 1)
		else:
			_push_position(xform, rope, lines, handles, flip, idx)

	if lines.size():
		gizmo.add_lines(lines, get_material("main", gizmo))
	if handles.size():
		gizmo.add_handles(handles, get_material(style, gizmo), [])
	if start.size():
		gizmo.add_handles(start, get_material("start_" + style, gizmo), [])
	if end.size():
		gizmo.add_handles(end, get_material("end_" + style, gizmo), [])

func _draw_attachments(gizmo: EditorNode3DGizmo, rope: Rope, start_path: NodePath, end_path: NodePath, attachments: RopeAttachmentsBase, style: StringName, flip: bool):
	var lines = PackedVector3Array()
	var handles = PackedVector3Array()
	var start = PackedVector3Array()
	var end = PackedVector3Array()
	
	var count := 0
	if attachments:
		count = attachments.get_count(rope)
		for idx in count:
			var path := attachments.get_nodepath(idx, rope)
			_push_positions_for_node(path, rope, lines, handles, flip, idx)
	
	_push_positions_for_node(start_path, rope, lines, start, flip, 0)
	_push_positions_for_node(end_path, rope, lines, end, flip, count - 1)
	
	if lines.size():
		gizmo.add_lines(lines, get_material("main", gizmo))
	if handles.size():
		gizmo.add_handles(handles, get_material(style, gizmo), [])
	if start.size():
		gizmo.add_handles(start, get_material("start_" + style, gizmo), [])
	if end.size():
		gizmo.add_handles(end, get_material("end_" + style, gizmo), [])

func _push_positions_for_node(path: NodePath, rope: Rope, lines: PackedVector3Array, handles: PackedVector3Array, flip: bool, index: int):
		var node: Node3D = rope.get_node_or_null(path)
		if node:
			_push_position(node.global_transform, rope, lines, handles, flip, index)

func _push_position(position: Transform3D, rope: Rope, lines: PackedVector3Array, handles: PackedVector3Array, flip: bool, index: int):
		var xform := rope.global_transform.inverse()
		var orientation := Transform3D(xform.basis, Vector3.ZERO)
		orientation = orientation.rotated_local(Vector3.RIGHT, PI / 4 + (PI / 32 * index))
		orientation = orientation.rotated_local(Vector3.UP, PI / 4 + (PI / 32 * index))
		var d := rope.get_rope_width() * 2
	
		var origin := position.origin
		var dir := Vector3.DOWN if flip else Vector3.UP
		lines.push_back(xform * origin)
		lines.push_back(xform * origin + d * dir * orientation)
		
		var handle_pos := xform * origin + d * 1.5 * dir * orientation
		handles.push_back(handle_pos)
