@tool
extends Node3D

@export_range(0, 20, 0.01) var coiled_length: float = 4.0:
	set(val):
		if coiled_length != val:
			coiled_length = val
			_dirty = true

@export var chain: Rope
@export var drum: RopeAnchor	
@export var coil: CoiledAnchor
@export var guide: RopeAnchor
@export var hook: RopeAnchor

@export_range(0,2.0, 0.01) var tension := 1.0:
	set(val):
		if tension != val:
			tension = val
			_dirty = true

@export var _debug: bool = false

var _dirty: bool = true

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	chain.clear_anchors()
	chain.anchor_count = 4
	chain.anchor_distribution = Rope.ABSOLUTE
	
	chain.set_anchor_nodepath(0, chain.get_path_to(drum))
	chain.set_anchor_offset(0, 0.0)
	
	chain.set_anchor_nodepath(1, chain.get_path_to(coil))
	chain.set_anchor_offset(1, coil.radius)
	
	chain.set_anchor_nodepath(2, chain.get_path_to(guide))
	chain.set_anchor_offset(2, coil.radius + coiled_length + 1.0)
	coil.coiled_length = coiled_length
	
	chain.set_anchor_nodepath(3, chain.get_path_to(hook))
	chain.set_anchor_from(3, Rope.End)
	chain.set_anchor_offset(3, 0.0)
	
	DebugDraw3D.scoped_config().set_thickness(0.001).set_center_brightness(0.6)


# Called every frame. 'delta' is the elapsed time since the previous frame.
var _lfo: float = 0.0
func _process(delta: float) -> void:
	if _dirty:
		_dirty = false
		
		var count := coil.get_particle_count()
		var last_pos := coil.get_particle_position(count-1)
		var global_pos := coil.to_global(last_pos)
		
		var guide_dist := global_pos.distance_to(guide.global_position) * tension
		coil.coiled_length = coiled_length
		chain.set_anchor_offset(2, coil.radius + coiled_length + guide_dist)
	
	var exit_angle := wrapf(coil.exit_angle + PI/2.0, -PI, PI)
	coil.rotation.x = lerp_angle(coil.rotation.x, exit_angle, delta*20)
	
	# modulate the coiled_length with an LFO
	if not Engine.is_editor_hint():
		_lfo = wrapf(_lfo + delta/3.0, 0, TAU)
		var wave := sin(_lfo)
		coil.coiled_length = wave * 5 + 10
	
	if _debug:
		for idx in chain.anchor_count:
			var xform := chain.get_anchor_transform(idx)
			var path := chain.get_anchor_nodepath(idx)
			var anchor: RopeAnchor = chain.get_node_or_null(path) as RopeAnchor
			if anchor:
				var particle_count := anchor.get_particle_count()
				for pos in particle_count:
					var color := Color.DARK_ORANGE if particle_count > 1 else Color.DARK_BLUE	
					var loc := xform * anchor.get_particle_position(pos)
					DebugDraw3D.draw_text(loc, str(pos), 8, color)
					DebugDraw3D.draw_square(loc, 0.025, color)
