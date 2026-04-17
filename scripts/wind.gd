@tool
extends Node
class_name Wind

signal wind_changed()

## Global wind controller. Sets global shader parameters for wind angle, speed, and gusts.

## Global Wind angle
@export_range(0,360, 1, "radians_as_degrees") var wind_angle: float = 0.0:
	get: return wind_angle
	set(value):
		wind_angle = value
		RenderingServer.global_shader_parameter_set("WindAngle", wind_angle)
		wind_changed.emit()


## Global Wind speed
@export_range(0,5,0.001, "m/s") var wind_speed: float = 0.1:
	get: return wind_speed
	set(value):
		wind_speed = value
		RenderingServer.global_shader_parameter_set("WindSpeed", wind_speed)
		wind_changed.emit()


## Global Wind gust strength
@export_range(0,5,0.001, "m/s") var wind_gust: float = 1.0:
	get: return wind_gust
	set(value):
		wind_gust = value
		RenderingServer.global_shader_parameter_set("WindGust", wind_gust)
		wind_changed.emit()


## Global Wind gust noise texture. Should be a seamless noise texture that scrolls in the wind direction.
@export var wind_noise: NoiseTexture2D = preload("res://addons/halyard/shaders/wind_noise.tres"):
	get: return wind_noise
	set(value):
		wind_noise = value
		RenderingServer.global_shader_parameter_set("WindMap", wind_noise)
		wind_changed.emit()

var wind_scroll: Vector2 = Vector2.ZERO:
	get: return wind_scroll
	set(value):
		wind_scroll = value
		RenderingServer.global_shader_parameter_set("WindScroll", wind_scroll)

# Retrieve the global wind controller. Assumed to be at root of tree, and cached for speed.
static var _wind: Wind = null
static func get_wind() -> Wind:
	if _wind == null or is_instance_valid(_wind) == false:
		var tree := Engine.get_main_loop() as SceneTree
		if tree:
			var node := tree.get_root().find_child("Wind", true, false)
			_wind = node
	
	return _wind

var _update: float = 0.0
func _process(delta: float) -> void:
	_update -= delta
	if _update < 0.0:
		_update = randfn(6, 5)
		
		# lerp the wind angle i a random direction
		var tween := create_tween()
		var new_angle := lerp_angle(wind_angle, wind_angle + randfn(0, PI/8.0), 1.0)
		tween.tween_property(self, "wind_angle", new_angle, 1.0)
	
	# animate the wind motion based upon the direction
	var movement := wind_scroll + get_wind_direction() * maxf(wind_speed/4.0, 0.001) * delta
	movement.x = wrapf(movement.x, 0, 1)
	movement.y = wrapf(movement.y, 0, 1)
	wind_scroll = movement
		

func get_wind_direction() -> Vector2:
	return Vector2(sin(wind_angle), cos(wind_angle))
