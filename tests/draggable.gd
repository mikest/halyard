@tool
extends RigidBody3D
class_name DraggableBody
## A RigidBody3D that can be picked up and dragged with the left mouse button.
## Uses force for movement.

#region Signals
signal drag_started
signal drag_updated(contact: Vector3)
signal drag_ended
#endregion


#region Properties
## How strongly the body chases the mouse while dragging.
@export var drag_strength: float = 20.0
## Damping applied while dragging to prevent oscillation.
@export var drag_damping: float = 5.0
## Extra spring gain based on distance
@export var drag_stiffness: float = 0.0
## Drag from the contact point or drag from the center of mass?
@export var drag_from_center: bool = false
## Draw debug
@export var debug: bool = false
#endregion


#region Initializers
var _dragging: bool = false
var _drag_depth: float = 0.0
var _target_position: Vector3 = Vector3.ZERO
var _local_contact_point: Vector3 = Vector3.ZERO
var _camera: Camera3D = null
#endregion


#region Runtime
func _ready() -> void:
	if Engine.is_editor_hint():
		return
	
	# Enable input events on this collision object.
	input_ray_pickable = true
	set_process_input(true)


func _input_event(camera: Camera3D, event: InputEvent, event_position: Vector3, _normal: Vector3, _shape_idx: int) -> void:
	if Engine.is_editor_hint():
		return
	
	# Begin dragging on left-click.
	if event is InputEventMouseButton:
		var mb: InputEventMouseButton = event as InputEventMouseButton
		if mb.button_index == MOUSE_BUTTON_LEFT and mb.pressed:
			_start_drag(camera, event_position)
			get_viewport().set_input_as_handled()


func _input(event: InputEvent) -> void:
	if Engine.is_editor_hint():
		return
		
	if not _dragging:
		return

	# Use _input (not _unhandled_input) while dragging so we consume events
	# before the camera orbit's _unhandled_input can steal them.

	# Stop dragging on left-button release.
	if event is InputEventMouseButton:
		var mb: InputEventMouseButton = event as InputEventMouseButton
		if mb.button_index == MOUSE_BUTTON_LEFT and not mb.pressed:
			_stop_drag()
			get_viewport().set_input_as_handled()

	# Update the target position as the mouse moves.
	if event is InputEventMouseMotion:
		_update_target_position(event as InputEventMouseMotion)
		get_viewport().set_input_as_handled()


func _integrate_forces(state: PhysicsDirectBodyState3D) -> void:
	if not _dragging:
		return

	if drag_from_center:
		# Damp force toward target position when dragging from center of mass.
		var delta_pos: Vector3 = _target_position - state.transform.origin
		var center_gain: float = _distance_stiffness_scale(delta_pos.length())
		var force: Vector3 = delta_pos * drag_strength * center_gain - state.linear_velocity * drag_damping
		
		state.apply_central_force(force * mass)
		drag_updated.emit(state.transform.origin)

	# Keep pulling the same point on the body's surface that was initially clicked.
	else:
		var contact_world: = state.transform * _local_contact_point
		
		var contact_offset: Vector3 = contact_world - state.transform.origin
		var delta_contact: Vector3 = _target_position - contact_world
		var contact_velocity: Vector3 = state.get_velocity_at_local_position(contact_offset)
		var contact_gain: float = _distance_stiffness_scale(delta_contact.length())
		var contact_force: Vector3 = delta_contact * drag_strength * contact_gain - contact_velocity * drag_damping
		
		state.apply_force(contact_force * mass, contact_offset)
		drag_updated.emit(contact_world)

		if debug:
			DebugDraw3D.draw_arrow(contact_world, _target_position, Color.RED, 0.1, true, 0.0001)
			DebugDraw3D.draw_square(state.transform.origin + contact_offset, 0.1, Color.CYAN, 0.0001)
			DebugDraw3D.draw_square(_target_position, 0.1, Color.GREEN, 0.0001)
			DebugDraw3D.draw_square(state.transform.origin + _local_contact_point, 0.3, Color.YELLOW, 0.0001)

			var intertia_tensor: Basis = state.inverse_inertia_tensor
			var torque: Vector3 = contact_offset.cross(contact_force)
			var angular_accel: Vector3 = intertia_tensor * torque
			DebugDraw3D.draw_arrow(state.transform.origin + state.center_of_mass, state.transform.origin + state.center_of_mass + angular_accel, Color.ORANGE, 0.1, true, 0.0001)
			DebugDraw3D.draw_arrow(state.transform.origin + state.center_of_mass, state.transform.origin + state.center_of_mass + state.angular_velocity, Color.PURPLE, 0.1, true, 0.0001)



func _get_configuration_warnings() -> PackedStringArray:
	var warnings: PackedStringArray = []
	return warnings


func _set(_property: StringName, _value: Variant) -> bool:
	update_configuration_warnings()
	return false
#endregion


#region Private Helpers
func _start_drag(cam: Camera3D, hit_position: Vector3) -> void:
	_camera = cam
	_dragging = true
	
	# Remember how far the hit point is from the camera so we drag on the same depth plane.
	_drag_depth = cam.global_position.distance_to(hit_position)
	_target_position = hit_position
	_local_contact_point = to_local(hit_position)
	
	# Prevent the body from sleeping while being dragged.
	sleeping = false
	drag_started.emit()


func _stop_drag() -> void:
	_dragging = false
	_camera = null
	drag_ended.emit()


func _update_target_position(motion: InputEventMouseMotion) -> void:
	if _camera == null:
		return
		
	# Project the mouse position onto a plane at the original drag depth.
	var from: Vector3 = _camera.project_ray_origin(motion.position)
	var dir: Vector3 = _camera.project_ray_normal(motion.position)
	_target_position = from + dir * _drag_depth


func _distance_stiffness_scale(distance: float) -> float:
	if drag_stiffness <= 0.0:
		return 1.0

	# distance_term grows slowly and steadily as the target gets farther away.
	var distance_term: float = log(1.0 + max(distance, 0.0))
	return 1.0 + drag_stiffness * distance_term
#endregion
