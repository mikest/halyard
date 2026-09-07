@tool
extends MeshInstance3D
class_name Pipe

@export var curve: Curve3D:
	set(val):
		curve = val
		_dirty = true

@export var elbow_radius: float = 1.0:
	set(val):
		elbow_radius = val
		_dirty = true

@export var elbow_length: float = 0.2:
	set(val):
		elbow_length = val
		_dirty = true
		
@export var elbow_scale: float = 1.1:
	set(val):
		elbow_scale = val
		_dirty = true

@export var flange_length: float = 0.2:
	set(val):
		flange_length = val
		_dirty = true

@export var flange_scale: float = 1.2:
	set(val):
		flange_scale = val
		_dirty = true

@export var elbow_material: Material:
	set(val):
		elbow_material = val
		_dirty = true

@export var pipe_material: Material:
	set(val):
		pipe_material = val
		_dirty = true

@export var rope_mesh: RopeMesh:
	set(val):
		rope_mesh = val
		_dirty = true

# internal mesh instance
var _frames: Array[Transform3D] = []
var _dirty: bool = false

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	mesh = rope_mesh

	# rebake on curve change
	if curve:
		curve.changed.connect(func ():
			_dirty = true
			)
	
	# cancel initial rebuild from dirty flag set by export inits
	_dirty = false
	
	# only bake if we have no existing surfaces
	if rope_mesh and rope_mesh.get_surface_count() == 0:
		_update_mesh()


func _process(delta: float) -> void:
	if _dirty:
		_build_frames()
		_update_mesh()
		_dirty = false
	
	#for frame in _frames:
		#DebugDraw3D.draw_gizmo(frame)
	pass


func _update_mesh():
	print("rebaking")
	if rope_mesh and curve:
		rope_mesh.clear_mesh();
		
		_build_frames()
		
		# pipe
		if _frames.size() >= 2:
			rope_mesh.begin_update_mesh();
			var first := _frames[0].rotated_local(Vector3.RIGHT, PI/2).orthonormalized()
			rope_mesh.emit_endcap(true, first);
			
			# emit sections for frames
			for idx in range(1, _frames.size()):
				var prev := _frames[idx-1].rotated_local(Vector3.RIGHT, PI/2).orthonormalized()
				var next := _frames[idx].rotated_local(Vector3.RIGHT, PI/2).orthonormalized()
				rope_mesh.emit_tube([prev, next]);
			
			var last := _frames[_frames.size()-1].rotated_local(Vector3.RIGHT, PI/2).orthonormalized()
			rope_mesh.emit_endcap(false, last);
			rope_mesh.end_update_mesh(null);
		
		rope_mesh.surface_set_material(0, pipe_material)
		
		# elbows
		if _frames.size() >= 2:
			var prev_elbow := false
			
			# emit sections for frames
			for idx in range(1, _frames.size()):
				var prev := _frames[idx-1].rotated_local(Vector3.RIGHT, PI/2)
				var next := _frames[idx].rotated_local(Vector3.RIGHT, PI/2)
				
				if not prev.basis.is_orthonormal() or not next.basis.is_orthonormal():
					if prev_elbow == false:
						rope_mesh.begin_update_mesh();
					rope_mesh.emit_tube([prev, next]);
					prev_elbow = true
				else:
					if prev_elbow == true:
						rope_mesh.end_update_mesh(null);
						rope_mesh.surface_set_material(rope_mesh.get_surface_count()-1, elbow_material)
					prev_elbow = false


func _build_frames() -> void:
	_frames.clear()
	
	# Need at least two points to calculate
	if curve and curve.point_count >= 2:
		
		# X right, Y up, -Z forward
		var basis_prev: Basis
		var frame_prev: Transform3D
		var tilt_prev: float = 0.0
		
		# Initial frame
		const UNIT_EPSILON := 0.001
		var first := curve.get_point_position(0)
		var second := curve.get_point_position(1)
		var forward := first.direction_to(second)
		if absf(forward.dot(Vector3.UP)) > 1.0 - UNIT_EPSILON:
			basis_prev = Basis.looking_at(forward, Vector3.RIGHT)
		else:
			basis_prev = Basis.looking_at(forward, Vector3.UP)
		
		var frame := Transform3D(basis_prev, first)
		frame = frame.rotated_local(Vector3.FORWARD, curve.get_point_tilt(0))
		_frames.push_back(frame)
		frame_prev = frame
		tilt_prev = curve.get_point_tilt(0)
		
		# Parallel transport through middle section
		for idx in range(1, curve.point_count-1):
			var prev := curve.get_point_position(idx-1)
			var curr := curve.get_point_position(idx)
			var next := curve.get_point_position(idx+1)
			
			var incoming := prev.direction_to(curr)
			var outgoing := curr.direction_to(next)
			
			var geo: ElbowGeometry = ElbowGeometry.new(elbow_radius, curr, incoming, outgoing)
			
			var flange_start := geo.tangent_start - incoming * (elbow_length + flange_length)
			var elbow_start := geo.tangent_start - incoming * (elbow_length)
			var elbow_end := geo.tangent_end + outgoing * elbow_length
			var flange_end := geo.tangent_end + outgoing * (elbow_length + flange_length)
			
			#DebugDraw3D.draw_square(geo.center)
			
			# prev to elbow start
			var tilt := lerpf(tilt_prev, curve.get_point_tilt(0), 0.5)
			frame = _calculate_frame(
				curve.get_point_position(idx-1),
				flange_start,
				tilt,
				basis_prev)
			_frames.push_back(frame.scaled_local(1.00 * Vector3.ONE))
			_frames.push_back(frame.scaled_local(flange_scale * Vector3.ONE))
			basis_prev = frame.basis
			frame_prev = frame
			tilt_prev = tilt
			
			# elbow sleeve
			frame = _calculate_frame(
				flange_start,
				elbow_start,
				tilt,
				basis_prev)
			_frames.push_back(frame.scaled_local(flange_scale * Vector3.ONE))
			_frames.push_back(frame.scaled_local(elbow_scale * Vector3.ONE))
			basis_prev = frame.basis
			frame_prev = frame
			
			# curved section
			var start := geo.tangent_start - geo.center
			var count := int(geo.angle/PI*2.0 * rope_mesh.sides)
			for rdx in range(0, count):
				var xform := Transform3D()
				xform = xform.translated(start)
				xform = xform.rotated(geo.normal, geo.angle/count * rdx + geo.angle/count * 0.5)
				xform = xform.translated(geo.center)
				
				frame = _calculate_frame(
					frame_prev.origin,
					xform.origin,
					tilt,
					basis_prev)
				
				_frames.push_back(frame.scaled_local(elbow_scale * Vector3.ONE))
				basis_prev = frame.basis
				frame_prev = frame
				pass
			
			# elbow sleeve
			frame = _calculate_frame(
				geo.tangent_end,
				elbow_end,
				tilt,
				basis_prev)
			_frames.push_back(frame.scaled_local(elbow_scale * Vector3.ONE))
			_frames.push_back(frame.scaled_local(flange_scale * Vector3.ONE))
			basis_prev = frame.basis
			frame_prev = frame
			
			# elbow end
			frame = _calculate_frame(
				elbow_end,
				flange_end,
				curve.get_point_tilt(idx),
				basis_prev)
			_frames.push_back(frame.scaled_local(flange_scale * Vector3.ONE))
			_frames.push_back(frame.scaled_local(1 * Vector3.ONE))
			basis_prev = frame.basis
			frame_prev = frame
			tilt_prev = curve.get_point_tilt(idx)
			
		# Last segment in frames
		var last := curve.point_count - 1
		first = _frames.back().origin
		second = curve.get_point_position(last)
		frame = _calculate_frame(
				_frames.back().origin,
				curve.get_point_position(last),
				curve.get_point_tilt(last),
				basis_prev)
		_frames.push_back(frame)
	pass


func _calculate_frame(first: Vector3, second: Vector3, tilt: float, basis_prev: Basis) -> Transform3D:
	var forward := first.direction_to(second)
	
	var rotate := _rotate_to_align(Basis(), -basis_prev.z, forward)
	var basis_cur := (rotate * basis_prev).orthonormalized()
	
	var frame := Transform3D(basis_cur, second)
	frame = frame.rotated_local(Vector3.FORWARD, tilt)
	return frame


# From godot:basis.cpp -> Basis::rotate_to_align
func _rotate_to_align(basis: Basis, start: Vector3, end: Vector3 ) -> Basis:
	var axis := start.cross(end).normalized()
	if axis.length_squared() != 0:
		var dot := start.dot(end)
		dot = clampf(dot, -1, 1)
		var angle := acos(dot)
		basis = Basis(axis, angle) * basis
	return basis

#region Support Class

# Calculates the points for an arc transcribing a pair of lines that meet at a point
# Used to remove sharps and create elbow bends.
class ElbowGeometry:
	var elbow_radius: float
	
	var basis: Basis
	var normal: Vector3
	var tangent: Vector3
	var binormal: Vector3
	var center: Vector3
	var tangent_start: Vector3
	var tangent_end: Vector3
	var angle : float
	
	func _init(radius: float, point: Vector3, incoming: Vector3, outgoing: Vector3):
		elbow_radius = radius
		
		# force normalized
		incoming = incoming.normalized()
		outgoing = outgoing.normalized()
		
		# calc normal for the plane
		normal = incoming.cross(outgoing)
		if normal.length_squared() < 0.000001:
			normal = _stable_reference_normal(incoming)
			normal = normal - incoming * incoming.dot(normal)
			normal = normal.normalized()
		else:
			normal = normal.normalized()
		
		# plane tangent and bitangent
		tangent = incoming.normalized()
		binormal = normal.cross(tangent).normalized()
		
		basis = Basis(tangent, normal, binormal)
		
		# ...uses plane_normal, plane_x, plane_y and elbow_radius
		center = _calculate_elbow_center(point, incoming, outgoing)
		
		# points on the lines where the curve starts/ends
		tangent_start = point + incoming * ((center - point).dot(incoming))
		tangent_end = point + outgoing * ((center - point).dot(outgoing))
		
		# angle of the sharp
		angle = acos(clampf(incoming.dot(outgoing), -1.0, 1.0))
	

	# Find the center of a circle transcribing two lines joined at a point in a 2D plane.
	func _calculate_elbow_center(point: Vector3, incoming: Vector3, outgoing: Vector3) -> Vector3:
		var bisector: Vector3 = (incoming + outgoing).normalized()
		var inward_normal_in: Vector3 = normal.cross(incoming).normalized()
		var inward_normal_out: Vector3 = normal.cross(outgoing).normalized()
		
		# this controls which side of the sharp our center is on.
		if inward_normal_in.dot(bisector) < 0.0:
			inward_normal_in = -inward_normal_in
		if inward_normal_out.dot(bisector) > 0.0:
			inward_normal_out = -inward_normal_out

		# The offset line is parallel to the original segment and sits elbow_radius away from it.
		# For any point on the original segment, the perpendicular distance to this line is elbow_radius.
		var incoming_offset_origin: Vector3 = point + inward_normal_in * elbow_radius
		var outgoing_offset_origin: Vector3 = point + inward_normal_out * elbow_radius
		var line_in_origin: Vector2 = Vector2(
			(incoming_offset_origin - point).dot(tangent),
			(incoming_offset_origin - point).dot(binormal)
		)
		var line_out_origin: Vector2 = Vector2(
			(outgoing_offset_origin - point).dot(tangent),
			(outgoing_offset_origin - point).dot(binormal)
		)
		var line_in_dir: Vector2 = Vector2(incoming.dot(tangent), incoming.dot(binormal))
		var line_out_dir: Vector2 = Vector2(outgoing.dot(tangent), outgoing.dot(binormal))
		var line_denominator: float = line_in_dir.cross(line_out_dir)
		if abs(line_denominator) <= 0.000001:
			line_denominator = 1.0

		var center_2d: Vector2 = line_in_origin + line_in_dir * ((line_out_origin - line_in_origin).cross(line_out_dir) / line_denominator)
		return point + tangent * center_2d.x + binormal * center_2d.y


	# util for avoiding degenerate normal
	func _stable_reference_normal(direction: Vector3) -> Vector3:
		if abs(direction.x) < 0.9:
			return Vector3.RIGHT
		return Vector3.UP

#region
