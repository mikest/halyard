# Frequently Asked Questions

### What level of compatibility/support do you offer?
None :sweat: This library is designed for use in my game and I am the lone target user. If it works for you, that's awesome! I will break compatibility frequently and often when adding _cool new features_.

If you would like to discuss some sort of support license agreement, feel free to reach out.

With that being said, I am interested in helping you get up and running, and am open to suggestions for improvements.

I always want to hear about bugs!


### Did you vibe code this library?
Claude is not very good at physics/computer graphics/complex algorithms and so unfortunately, _all the garbage code you find in here has been lovelingly hand-crafted by me_. I appologize in advance for it.

However, because I value my time, I still use Claude for rote boilerplate and roughing out classes.

If this is a problem for you, know that I am not interested in hearing your opinions on "why AI is evil." I probably already agree with you on most points anyway.

All the art assets in Halyard are made by humans, and will always be made by humans.


### Do you accept vibe-coded PRs?
Unfortunately, I neither have the time or the patience to do forensic analysis on your PR, so here are three rules I will apply:
- If you can't explain your code, I will reject it.
- If I can tell you vibe coded it, I will reject it.
- If you send me _obvious slop_, I will reject it, and then probably block you.


### How many floating objects can this support?
 Depends a lot on how they're rigged, and what kind of buoyancy modes your are using. For 3-probe simple point buoyancy you can expect 100's of objects. For mesh buoyancy you can expect 10s of objects.

 
 ### Why does nothing move with my waves?
 You gave to connect your GPU wave system back to the CPU physics. This probably involves replicating your wave mesh calculations CPU side, or doing an asyn readback from the GPU for your calculated mesh. I can not help you with this.

 See documention for LiquidArea.


### Why doesn't CharacterBuoyancy apply torque?
Character controllers work differently than RigidBody3D based objects, and are usually under some kind of script control for their velocities/angles.


### How do I get ropes to pull on attached objects?
Place a `RopeAnchor` as a child of a `RigidBody` and set the `behavior` to `Towing`. Connect your rop to that anchor. Move the other end of the rope.


### My rope is too weak / doesn't pull hard enough on attached bodies
Increase `tension_force_scale` to strengthen the rope-body coupling:
- Default is `1.0` for balanced behavior
- Try `2.0-5.0` for stronger, more rigid rope-like behavior
- If still too weak, also increase `max_tension_force` from the default `1000.0` to a higher value like `5000.0` or more

You may also want to increase `stiffness` and stiffness iterations to make the rope less stretchy.


### My attached bodies are flying apart / simulation is unstable
Lower the `max_tension_force` to prevent explosive forces:
- Default is `1000.0`, try reducing to `500.0` or `100.0`
- Or reduce `tension_force_scale` to `0.5` or lower for softer coupling
- Also check that `stiffness` isn't too high (should be < 2.0)
- Increase `stiffness_iterations` (default: 2) for better stability with higher stiffness


### Can I disable force feedback to attached bodies?
Yes, set the behavior to `ANCHORED` or `FREE` depending on the desired outcome.


## Why are my ropes exploding all over the place?
Check your collision layer interactions. Colliding with an object that the rope is towing or sliding along has unpredictable behavior.