# Deterministic Velcro Physics

## What is this?
Velcro Physics is a high performance 2D collision detection system with realistic physics responses.
This fork changed all floating-point operations into fixed-point operations making the engine simulate deterministically across all platforms.
It's using modified [Fix64](https://github.com/asik/FixedMath.Net) as a math basis.

## What is it good for?
This is perfect for simulations synchronized over network with lockstep, as the simulation will behave identically with the same starting state.

## Features
For physics features check out [VelcroPhysics](https://github.com/VelcroPhysics/VelcroPhysics).

## Integration
This version was developed in Unity3D environment, but you can run it anywhere else.
