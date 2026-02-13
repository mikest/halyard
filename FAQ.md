# Frequently Asked Questions

### How many floating objects can this support?
 Depends a lot on how they're rigged, and what kind of buoyancy modes your are using. For 3-probe simple point buoyancy you can expect 100's of objects. For mesh buoyancy you can expect 10s of objects.

 
 ### Why does nothing move with my waves?

 You gave to connect your GPU wave system back to the CPU physics. This probably involves replicating your wave mesh calculations CPU side, or doing an asyn readback from the GPU for your calculated mesh. I can not help you with this.

 See documention for LiquidArea.


### Why doesn't CharacterBuoyancy apply torque?

Character controllers work differently than RigidBody3D based objects, and are usually under some kind of scrip control for their velocities.


### How do I get ropes to pull on attached objects?

Ropes automatically apply tension forces to attached RigidBody3D nodes. Simply anchor your rope to a RigidBody3D using the `start_anchor` or `end_anchor` properties:

```gdscript
$Rope.start_anchor = get_path_to($RigidBody3D)
```

The rope will automatically detect the RigidBody3D and apply constraint forces during simulation. Use `tension_force_scale` to control the strength of the coupling and `max_tension_force` to prevent instability.


### My rope is too weak / doesn't pull hard enough on attached bodies

Increase `tension_force_scale` to strengthen the rope-body coupling:
- Default is `1.0` for balanced behavior
- Try `2.0-5.0` for stronger, more rigid rope-like behavior
- If still too weak, also increase `max_tension_force` from the default `1000.0` to a higher value like `5000.0` or more

You may also want to increase `stiffness` (default: `0.9`) to make the rope less stretchy.


### My attached bodies are flying apart / simulation is unstable

Lower the `max_tension_force` to prevent explosive forces:
- Default is `1000.0`, try reducing to `500.0` or `100.0`
- Or reduce `tension_force_scale` to `0.5` or lower for softer coupling
- Also check that `stiffness` isn't too high (should be < 2.0)
- Increase `stiffness_iterations` (default: 2) for better stability with higher stiffness


### Can I disable force feedback to attached bodies?

Yes, set `tension_force_scale` to `0.0` to disable force feedback entirely. The rope will still follow anchored nodes, but won't apply any reaction forces back to them.


## Why are my rope exploding all over the place?

Check your collision layer interactions.