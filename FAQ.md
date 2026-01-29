# Frequently Asked Questions

### How many floating objects can this support?
 Depends a lot on how they're rigged, and what kind of buoyancy modes your are using. For 3-probe simple point buoyancy you can expect 100's of objects. For mesh buoyancy you can expect 10s of objects.

 
 ### Why does nothing move with my waves?

 You gave to connect your GPU wave system back to the CPU physics. This probably involves replicating your wave mesh calculations CPU side, or doing an asyn readback from the GPU for your calculated mesh. I can not help you with this.

 See documention for LiquidArea.


### Why doesn't CharacterBuoyancy apply torque?

Character controllers work differently than RigidBody3D based objects, and are usually under some kind of scrip control for their velocities.


## Why are my rope exploding all over the place?

Check your collision layer interactions.