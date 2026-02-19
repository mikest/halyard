@tool
extends LiquidArea
class_name LiquidShape
## A LiquidArea that determines the liquid surface via raycasts against a physics layer.
##
## First tests if point is inside a collision shape in [member liquid_mask]
##
## Then raycasts downward from above each query point, using [member liquid_mask]
## to filter which collision objects define the surface. The rays span
## [member liquid_range] above and below this node's origin on the Y axis.
##


## Physics collision mask used to detect liquid surface geometry.
@export_flags_3d_physics var liquid_mask: int = 1

## How far above and below the origin (in meters) the raycasts extend.
@export var liquid_range: float = 10.0


# cached direct space query
var _raycast: PhysicsRayQueryParameters3D
var _contains: PhysicsPointQueryParameters3D

#region Runtime
func _ready() -> void:
	if Engine.is_editor_hint(): return

	# we use these a lot, so cache them.
	_raycast = PhysicsRayQueryParameters3D.create(Vector3.ONE, Vector3.ZERO, liquid_mask)
	_contains = PhysicsPointQueryParameters3D.new()
	_contains.collide_with_bodies = true
	_contains.collision_mask = liquid_mask


func _get_configuration_warnings() -> PackedStringArray:
	var warnings: PackedStringArray = []
	if liquid_mask == 0:
		warnings.append("liquid_mask is 0 — no collision layers will be tested.")
	return warnings


func _set(_property: StringName, _value: Variant) -> bool:
	update_configuration_warnings()
	return false
#endregion


#region Overrides
func update_transforms_for_points(global_points: PackedVector3Array,
		transforms: Array[Transform3D]) -> void:
	
	var no_hit := Transform3D(Basis(), Vector3(0,-1000,0))
	
	## Cast a ray downward at each query point to find the liquid surface.
	# NOTE: is this even necessary? when would the world not be present?
	var space_state: PhysicsDirectSpaceState3D = get_world_3d().direct_space_state
	if space_state == null:
		transforms.resize(global_points.size())
		transforms.fill(no_hit)
		return

	var origin_y: float = global_position.y
	var ray_top: float = origin_y + liquid_range
	var ray_bottom: float = origin_y - liquid_range
	
	transforms.resize(global_points.size())
	
	for idx in global_points.size():
		var point: Vector3 = global_points[idx]

		# contact collision test first
		_contains.position = point
		
		var contains_result: Array[Dictionary] = space_state.intersect_point(_contains)
		if contains_result.size() <= 0:
			# no hits, return transform at point
			transforms[idx] = no_hit
			continue

		# Ray from above the range straight down to below the range.
		_raycast.from = Vector3(point.x, ray_top, point.z)
		_raycast.to = Vector3(point.x, ray_bottom, point.z)
		
		var result: Dictionary = space_state.intersect_ray(_raycast)
		if result.size() > 0:
			# Build a transform at the hit point with the surface normal as Y-up.
			var hit_pos: Vector3 = result["position"]
			var hit_normal: Vector3 = result["normal"]
			var basis: Basis = _basis_from_normal(hit_normal)
			transforms[idx] = Transform3D(basis, hit_pos)
		else:
			transforms[idx] = no_hit
#endregion


#region Private Helpers
func _basis_from_normal(normal: Vector3) -> Basis:
	# Build an oriented basis where Y aligns with the given normal.
	var up: Vector3 = normal.normalized()
	
	# Pick a reference vector that isn't parallel to the normal.
	var ref: Vector3 = Vector3.FORWARD if absf(up.dot(Vector3.FORWARD)) < 0.99 else Vector3.RIGHT
	var right: Vector3 = up.cross(ref).normalized()
	var forward: Vector3 = right.cross(up).normalized()
	
	return Basis(right, up, forward)
#endregion
