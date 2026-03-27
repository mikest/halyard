@tool
extends HBoxContainer

var plugin: EditorPlugin
var menu_button: MenuButton = MenuButton.new()

enum {
	MENU_BAKE_INITIAL_POS,
	MENU_CLEAR_INITIAL_POS,
	MENU_SET_ATTACHMENTS_FROM_CHILDREN,
	MENU_CLEAR_ATTACHMENTS,
	MENU_CLEAR_ANCHORS,
	MENU_MAX,
}

func _enter_tree() -> void:
	menu_button.text = "Rope"
	menu_button.icon = preload("../icons/Rope.svg")
	menu_button.add_theme_constant_override("icon_max_width", 20)
	
	menu_button.get_popup().add_item("Bake Initial Positions", MENU_BAKE_INITIAL_POS)
	menu_button.get_popup().add_item("Clear Initial Positions", MENU_CLEAR_INITIAL_POS)
	menu_button.get_popup().add_separator()
	menu_button.get_popup().add_item("Set Attachments From Children", MENU_SET_ATTACHMENTS_FROM_CHILDREN)
	menu_button.get_popup().add_item("Clear Attachments", MENU_CLEAR_ATTACHMENTS)
	menu_button.get_popup().add_separator()
	menu_button.get_popup().add_item("Clear Anchors", MENU_CLEAR_ANCHORS)
	menu_button.get_popup().id_pressed.connect(_on_menu_pressed)
	menu_button.about_to_popup.connect(_on_menu_about_to_popup)
	add_child(menu_button)
	pass


func _on_menu_pressed(p_id: int) -> void:
	match p_id:
		MENU_BAKE_INITIAL_POS:
			if plugin.current_rope:
				plugin.current_rope._bake_initial_pos()
		MENU_CLEAR_INITIAL_POS:
			if plugin.current_rope:
				plugin.current_rope._set_initial_pos([])
		MENU_SET_ATTACHMENTS_FROM_CHILDREN:
			if plugin.current_rope:
				plugin.current_rope._set_attachments_from_children()
		MENU_CLEAR_ATTACHMENTS:
			if plugin.current_rope and plugin.current_rope.appearance:
				plugin.current_rope.appearance.clear_attachments()
			pass
		MENU_CLEAR_ANCHORS:
			if plugin.current_rope:
				plugin.current_rope.clear_anchors()

	pass


func _on_menu_about_to_popup() -> void:
	var no_rope := plugin.current_rope == null
	for idx in MENU_MAX:
		menu_button.get_popup().set_item_disabled(idx, no_rope)

	# has rope
	var rope: Rope = plugin.current_rope
	if rope:
		menu_button.get_popup().set_item_disabled(MENU_CLEAR_INITIAL_POS, rope._get_initial_pos().size() == 0)
		
		var can_attach := rope.get_child_count() > 0
		menu_button.get_popup().set_item_disabled(MENU_SET_ATTACHMENTS_FROM_CHILDREN, not can_attach)
		
		var can_clear := rope.appearance and rope.appearance.attachment_count > 0
		menu_button.get_popup().set_item_disabled(MENU_CLEAR_ATTACHMENTS, not can_clear)
		
		menu_button.get_popup().set_item_disabled(MENU_CLEAR_ANCHORS, rope.anchor_count == 0)
