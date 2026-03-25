@tool
extends HBoxContainer

var plugin: EditorPlugin
var menu_button: MenuButton = MenuButton.new()

enum {
	MENU_BAKE_INITIAL_POS,
	MENU_CLEAR_INITIAL_POS,
}

func _enter_tree() -> void:
	menu_button.text = "Rope"
	menu_button.icon = preload("../icons/Rope.svg")
	menu_button.add_theme_constant_override("icon_max_width", 20)
	
	menu_button.get_popup().add_item("Bake Initial Positions", MENU_BAKE_INITIAL_POS)
	menu_button.get_popup().add_item("Clear Initial Positions", MENU_CLEAR_INITIAL_POS)
	
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

	pass


func _on_menu_about_to_popup() -> void:
	menu_button.get_popup().set_item_disabled(MENU_BAKE_INITIAL_POS, not plugin.current_rope)
	menu_button.get_popup().set_item_disabled(MENU_CLEAR_INITIAL_POS, not plugin.current_rope or (plugin.current_rope and plugin.current_rope._get_initial_pos().size() == 0))
