@tool
## Editor plugin that adds a Rope toolbar menu to the 3D editor when a Rope node is selected.
extends EditorInspectorPlugin

func _can_handle(object):
	return (object as Rope) != null

func _init() -> void:
	pass
