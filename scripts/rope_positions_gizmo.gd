@tool
extends EditorNode3DGizmoPlugin

const anchor_icon := preload("res://addons/halyard/icons/Anchor.svg")
const attach_icon := preload("res://addons/halyard/icons/Attachment.svg")
const guided_icon := preload("res://addons/halyard/icons/Guided.svg")
const sliding_icon := preload("res://addons/halyard/icons/Sliding.svg")
const towing_icon := preload("res://addons/halyard/icons/Towing.svg")

const start_icon := preload("res://addons/halyard/icons/Start.svg")
const end_icon := preload("res://addons/halyard/icons/End.svg")

var lines := PackedVector3Array()
var attaches := PackedVector3Array()
var anchors := PackedVector3Array()
var guideds := PackedVector3Array()
var slidings := PackedVector3Array()
var towings := PackedVector3Array()
var starts := PackedVector3Array()
var ends := PackedVector3Array()

func _get_gizmo_name():
	return "RopePositions"

func _has_gizmo(node):
	return node is Rope

func _init():
	create_handle_material("attach", false, attach_icon)

	create_handle_material("anchor", false, anchor_icon)
	create_handle_material("guided", false, guided_icon)
	create_handle_material("sliding", false, sliding_icon)
	create_handle_material("towing", false, towing_icon)
	
	create_handle_material("start", false, start_icon)
	create_handle_material("end", false, end_icon)

	create_material("main", Color(1, 1, 0))


func _redraw(gizmo: EditorNode3DGizmo):
	gizmo.clear()
	
	lines.clear()
	attaches.clear()
	anchors.clear()
	guideds.clear()
	slidings.clear()
	towings.clear()
	starts.clear()
	ends.clear()

	var rope: Rope = gizmo.get_node_3d()
	if rope:
		_draw_anchors(gizmo, rope, false)
		_draw_attachments(gizmo, rope, true)
	
	# draw accumulated icons and lines
	if lines.size(): gizmo.add_lines(lines, get_material("main", gizmo))
	if attaches.size(): gizmo.add_handles(attaches, get_material("attach", gizmo), [])
	if anchors.size(): gizmo.add_handles(anchors, get_material("anchor", gizmo), [])
	if guideds.size(): gizmo.add_handles(guideds, get_material("guided", gizmo), [])
	if slidings.size(): gizmo.add_handles(slidings, get_material("sliding", gizmo), [])	
	if towings.size(): gizmo.add_handles(towings, get_material("towing", gizmo), [])
	if starts.size(): gizmo.add_handles(starts, get_material("start", gizmo), [])
	if ends.size(): gizmo.add_handles(ends, get_material("end", gizmo), [])

func _draw_anchors(gizmo: EditorNode3DGizmo, rope: Rope, flip: bool):
	var count := rope._get_anchor_count()
	for idx in count:
		var xform := rope._get_anchor_transform(idx)
		var behavior := rope._get_anchor_behavior(idx)
		match behavior:
			RopeAnchor.AnchorBehavior.ANCHORED:
				_push_position(xform, rope, lines, anchors, flip, idx)
			RopeAnchor.AnchorBehavior.GUIDED:
				_push_position(xform, rope, lines, guideds, flip, idx)
			RopeAnchor.AnchorBehavior.SLIDING:
				_push_position(xform, rope, lines, slidings, flip, idx)
			RopeAnchor.AnchorBehavior.TOWING:
				_push_position(xform, rope, lines, towings, flip, idx)
		if idx == 0:
			_push_position(xform, rope, lines, starts, flip, idx)
		if idx == count-1:
			_push_position(xform, rope, lines, ends, flip, idx)


func _draw_attachments(gizmo: EditorNode3DGizmo, rope: Rope, flip: bool):
	var count := 0
	var appearance: RopeAppearance = rope.get_appearance()
	if appearance:
		count = appearance.attachment_count
		for idx in count:
			var path := appearance.get_attachment_nodepath(idx)
			var node: Node3D = rope.get_node_or_null(path)
			if node:
				var xform := node.global_transform
				_push_position(xform, rope, lines, attaches, flip, idx)
				if idx == 0:
					_push_position(xform, rope, lines, starts, flip, idx)
				if idx == count-1:
					_push_position(xform, rope, lines, ends, flip, idx)


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
