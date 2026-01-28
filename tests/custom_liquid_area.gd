@tool
extends LiquidArea
class_name CustomLiquidArea

## This is a dirty and cheap height map to demonstrate "waves"
## You will probably want to work your own wave mechanics into
## A custom LiquidArea


## This is the noise texture applied to the mesh instance
@export var normal: NoiseTexture2D = null
@export var height: NoiseTexture2D = null
@export var shader: ShaderMaterial = null

## This is the mesh in the mesh instance
@export var mesh: PlaneMesh = null

@export_custom(PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR | PROPERTY_USAGE_NO_INSTANCE_STATE) \
var phase: float = 1.0

# cached image of the normal map
var _normal: Image
var _height: Image

func _ready() -> void:
	if Engine.is_editor_hint(): return
	pass
	

func _process(delta: float) -> void:
	if Engine.is_editor_hint():
		if shader:
			shader.set_shader_parameter("phase",  0)
		return
	
	# fetch once on first draw
	if normal and not _normal:
		_normal = normal.get_image()
	
	if height and not _height:
		_height = height.get_image()
	
	phase += delta / 10
	if phase > 1.0:
		phase = 0.0
	if shader:
		shader.set_shader_parameter("phase",  sin(phase*TAU))


func align_with_normal(normal: Vector3) -> Basis:
	var new_basis = Basis()
	new_basis.y = normal.normalized()  # Align Y axis with the normal
	new_basis.x = new_basis.y.cross(Vector3(0, 0, 1)).normalized()  # X axis perpendicular to Y and Z
	new_basis.z = new_basis.x.cross(new_basis.y).normalized()  # Z axis perpendicular to X and Y
	return new_basis.orthonormalized()  # Ensure the basis is orthonormal


func update_transforms_for_points(global_points: PackedVector3Array, transforms: Array[Transform3D]):
	# this line is important!
	transforms.resize(global_points.size());
	
	if not _normal or not _height:
		return Transform3D.IDENTITY
	
	# sample the normal and height map to get the ocean surface
	var img_width = _normal.get_width()
	var img_height = _normal.get_height()
	for idx in global_points.size():
		
		var pos = global_points[idx]
		var uv = Vector2(
			fposmod(pos.x * 0.1, 1.0),
			fposmod(pos.z * 0.1, 1.0)
		)
		var img_x = int(uv.x * float(img_width - 1))
		var img_y = int(uv.y * float(img_height - 1))
		var normal_color = _normal.get_pixel(img_x, img_y)
		var normal = Vector3(
			normal_color.r * 2.0 - 1.0,
			normal_color.b * 2.0 - 1.0,		# NOTE: B is Y in a Normal Map
			normal_color.g * 2.0 - 1.0,
			
		).normalized()
		
		var height_color = _height.get_pixel(img_x, img_y)
		pos.y = height_color.r * 4.0 * sin(phase*TAU)		# boost the hieght a to make "waves" taller

		var wave_basis = align_with_normal(normal)
		transforms[idx] = Transform3D(wave_basis, pos)
