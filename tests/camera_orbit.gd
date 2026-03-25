@tool
extends Camera3D
class_name CameraOrbit
## A camera that orbits the scene origin when the middle mouse button is held.
## Also zooms with the wheel.

#region Properties
## The point the camera orbits around.
@export var pivot: Vector3 = Vector3.ZERO
## Distance from the pivot point.
@export var orbit_distance: float = 5.0
## Mouse sensitivity in degrees per pixel.
@export var sensitivity: float = 0.3
## Minimum vertical angle (degrees) to prevent flipping.
@export var min_pitch: float = -89.0
## Maximum vertical angle (degrees) to prevent flipping.
@export var max_pitch: float = 89.0
## Scroll-wheel zoom speed.
@export var zoom_speed: float = 0.5
## Minimum orbit distance.
@export var min_distance: float = 1.0
## Maximum orbit distance.
@export var max_distance: float = 50.0
## Invert horizontal (yaw) mouse direction.
@export var invert_yaw: bool = false
## Invert vertical (pitch) mouse direction.
@export var invert_pitch: bool = false
## Pan speed in world units per pixel.
@export var pan_speed: float = 0.005
#endregion


#region Initializers
var _yaw: float = 0.0
var _pitch: float = -30.0
var _orbiting: bool = false
var _panning: bool = false
#endregion


#region Runtime
func _ready() -> void:
	if Engine.is_editor_hint():
		return
	
	# Initialise yaw / pitch from the current transform so the camera doesn't jump.
	var offset: Vector3 = global_position - pivot
	orbit_distance = offset.length()
	_yaw = rad_to_deg(atan2(offset.x, offset.z))
	_pitch = rad_to_deg(asin(clampf(offset.y / orbit_distance, -1.0, 1.0)))
	_apply_orbit()
	
	# so our input processing happens last
	process_priority = 99


func _unhandled_input(event: InputEvent) -> void:
	if Engine.is_editor_hint():
		return

	# Start / stop orbiting on middle or left mouse button.
	if event is InputEventMouseButton:
		var mb: InputEventMouseButton = event as InputEventMouseButton
		if mb.button_index == MOUSE_BUTTON_MIDDLE:
			# Shift + middle mouse = pan; plain middle mouse = orbit.
			if mb.pressed:
				_panning = mb.shift_pressed
				_orbiting = not mb.shift_pressed
			else:
				_orbiting = false
				_panning = false
			get_viewport().set_input_as_handled()

		# Zoom with scroll wheel.
		if mb.button_index == MOUSE_BUTTON_WHEEL_UP:
			orbit_distance = maxf(min_distance, orbit_distance - zoom_speed)
			_apply_orbit()
			get_viewport().set_input_as_handled()
		elif mb.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			orbit_distance = minf(max_distance, orbit_distance + zoom_speed)
			_apply_orbit()
			get_viewport().set_input_as_handled()
	
	# Trackpad
	if event is InputEventMagnifyGesture:
		orbit_distance = maxf(min_distance, orbit_distance * event.factor)
		_apply_orbit()
		get_viewport().set_input_as_handled()
	
	if event is InputEventPanGesture:
		var motion := event as InputEventPanGesture
		var yaw_sign: float = -1.0 if not invert_yaw else 1.0
		var pitch_sign: float = -1.0 if not invert_pitch else 1.0
		_yaw += motion.delta.x * sensitivity * yaw_sign
		_pitch += motion.delta.y * sensitivity * pitch_sign
		_pitch = clampf(_pitch, min_pitch, max_pitch)
		_apply_orbit()
		get_viewport().set_input_as_handled()
		

	# Rotate while orbiting.
	if event is InputEventMouseMotion and _orbiting:
		var motion := event as InputEventMouseMotion
		var yaw_sign: float = -1.0 if not invert_yaw else 1.0
		var pitch_sign: float = -1.0 if not invert_pitch else 1.0
		_yaw += motion.relative.x * sensitivity * yaw_sign
		_pitch += motion.relative.y * sensitivity * pitch_sign
		_pitch = clampf(_pitch, min_pitch, max_pitch)
		_apply_orbit()
		get_viewport().set_input_as_handled()

	# Pan the pivot while shift-orbiting.
	if event is InputEventMouseMotion and _panning:
		var motion := event as InputEventMouseMotion
		# Pan in camera-local right and up directions so it matches the view.
		var scale: float = orbit_distance * pan_speed
		pivot -= global_transform.basis.x * motion.relative.x * scale
		pivot += global_transform.basis.y * motion.relative.y * scale
		_apply_orbit()
		get_viewport().set_input_as_handled()
#endregion


#region Private Helpers
func _apply_orbit() -> void:
	# Convert spherical coordinates to a position offset from the pivot.
	var yaw_rad: float = deg_to_rad(_yaw)
	var pitch_rad: float = deg_to_rad(_pitch)
	var offset: Vector3 = Vector3(
		sin(yaw_rad) * cos(pitch_rad),
		sin(pitch_rad),
		cos(yaw_rad) * cos(pitch_rad)
	) * orbit_distance
	
	global_position = pivot + offset
	look_at(pivot, Vector3.UP)
#endregion
