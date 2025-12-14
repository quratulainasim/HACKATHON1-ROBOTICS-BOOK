---
title: Physics Simulation - Collisions, Gravity, Friction
sidebar_position: 2
---

# Physics Simulation - Collisions, Gravity, Friction

## Understanding Physics Simulation in Robotics

Physics simulation is fundamental to robotics as it allows us to model how robots interact with the physical world. Realistic physics simulation helps us develop, test, and validate robot behaviors before deploying them in the real world. This section covers the core physics concepts that underpin robot simulation environments.

## Core Physics Concepts

### Gravity

Gravity is the fundamental force that affects all objects with mass. In simulation, we must accurately model gravitational effects to ensure realistic robot behavior.

```xml
<!-- In Gazebo world files, gravity is defined globally -->
<sdf version="1.6">
  <world name="default">
    <gravity>0 0 -9.8</gravity>
    <!-- Other world elements -->
  </world>
</sdf>
```

For robots operating in different environments (Moon, Mars, underwater), the gravity vector needs adjustment:

- Earth: [0, 0, -9.8] m/s²
- Moon: [0, 0, -1.6] m/s²
- Mars: [0, 0, -3.7] m/s²

### Collisions and Contact Dynamics

Collision detection and response are critical for realistic simulation. There are two main aspects:

1. **Collision Detection**: Determining when and where objects intersect
2. **Collision Response**: Calculating the resulting forces and motions

#### Types of Collisions

**Static vs. Dynamic Collisions:**
- Static: One object is fixed (e.g., ground plane)
- Dynamic: Both objects can move and respond to forces

**Elastic vs. Inelastic Collisions:**
- Elastic: Objects bounce with preserved kinetic energy
- Inelastic: Objects may stick together or lose energy

#### Collision Properties

In Gazebo and similar simulators, collision properties are defined in the URDF/SDF:

```xml
<link name="collision_link">
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
  </inertial>
</link>
```

## Friction Models

Friction is the force that resists relative motion between surfaces in contact. In robotics, friction is essential for:
- Gripping objects
- Walking/locomotion
- Preventing unwanted sliding

### Coulomb Friction Model

The most common friction model in simulation:

```
F_friction = μ * F_normal
```

Where:
- `F_friction` is the frictional force
- `μ` is the coefficient of friction
- `F_normal` is the normal force between surfaces

#### Static vs. Dynamic Friction

- **Static friction (μ_s)**: Prevents initial motion; generally higher than dynamic friction
- **Dynamic friction (μ_d)**: Acts during motion; typically lower than static friction

```xml
<gazebo reference="wheel_link">
  <mu1>10.0</mu1>  <!-- Primary friction coefficient -->
  <mu2>10.0</mu2>  <!-- Secondary friction coefficient -->
  <kp>10000000.0</kp>  <!-- Contact stiffness -->
  <kd>1.0</kd>  <!-- Contact damping -->
</gazebo>
```

## Simulation Engines

### Open Dynamics Engine (ODE)

One of the earliest and most widely used physics engines:

**Advantages:**
- Fast computation
- Stable for many robotics applications
- Good for rigid body simulation

**Disadvantages:**
- Can be unstable with certain configurations
- Less accurate for soft body simulation

### Bullet Physics

A more modern engine with better stability:

**Advantages:**
- More stable than ODE
- Better collision detection
- Supports soft body simulation

**Disadvantages:**
- Slightly slower than ODE
- More complex configuration

### Simbody

Developed specifically for biomechanics and robotics:

**Advantages:**
- Very accurate for complex articulated systems
- Excellent for humanoid robots
- Handles closed-loop mechanisms well

**Disadvantages:**
- More computationally intensive
- Steeper learning curve

## Gazebo Physics Configuration

### World Physics Settings

```xml
<world name="my_world">
  <physics type="ode">
    <max_step_size>0.001</max_step_size>  <!-- Simulation time step -->
    <real_time_factor>1</real_time_factor>  <!-- Real-time vs. simulation time -->
    <real_time_update_rate>1000</real_time_update_rate>  <!-- Hz -->
    <gravity>0 0 -9.8</gravity>
    <ode>
      <solver>
        <type>quick</type>  <!-- Solver type -->
        <iters>10</iters>   <!-- Iterations per step -->
        <sor>1.3</sor>      <!-- Successive over-relaxation parameter -->
      </solver>
      <constraints>
        <cfm>0.0</cfm>      <!-- Constraint force mixing -->
        <erp>0.2</erp>      <!-- Error reduction parameter -->
        <contact_max_correcting_vel>100</contact_max_correcting_vel>
        <contact_surface_layer>0.001</contact_surface_layer>
      </constraints>
    </ode>
  </physics>
</world>
```

### Per-Link Physics Properties

```xml
<gazebo reference="my_link">
  <!-- Damping to simulate viscous friction -->
  <damping>
    <linear>0.01</linear>
    <angular>0.01</angular>
  </damping>

  <!-- Additional inertia properties -->
  <implicit_spring_damper>true</implicit_spring_damper>

  <!-- Contact properties -->
  <mu1>1.0</mu1>
  <mu2>1.0</mu2>
  <kp>10000000.0</kp>  <!-- Spring stiffness -->
  <kd>1.0</kd>         <!-- Damping coefficient -->
</gazebo>
```

## Collision Detection Strategies

### Discrete vs. Continuous Collision Detection

**Discrete Collision Detection:**
- Checks for collisions at fixed time intervals
- Faster but can miss collisions with fast-moving objects
- Suitable for most robotics applications

**Continuous Collision Detection:**
- Predicts collisions between time steps
- More accurate but computationally expensive
- Necessary for very fast-moving objects

### Collision Primitives

Common geometric shapes used for collision detection:

1. **Box**: Simple rectangular shapes
2. **Sphere**: Perfect for round objects, computationally efficient
3. **Cylinder**: Good for wheels, limbs
4. **Mesh**: Complex shapes but computationally expensive

## Realistic Physics for Robotics Applications

### Manipulation Tasks

For robot manipulation, accurate physics simulation includes:

- **Friction coefficients** for grasping and manipulation
- **Surface properties** affecting grip quality
- **Mass properties** of objects being manipulated
- **Inertia tensors** for dynamic manipulation

### Locomotion

For legged robots, physics simulation must account for:

- **Foot-ground contact** with appropriate friction
- **Dynamic balance** and center of mass tracking
- **Impact forces** during foot strikes
- **Energy dissipation** in compliant contacts

### Multi-Robot Systems

When simulating multiple robots:

- **Collision avoidance** between robots
- **Communication delays** affecting coordination
- **Shared environment** effects
- **Resource competition** (e.g., for workspace)

## Performance Considerations

### Time Step Selection

The simulation time step affects both accuracy and performance:

- **Smaller steps**: More accurate but slower
- **Larger steps**: Faster but potentially unstable
- **Rule of thumb**: At least 10x the control frequency

### Optimization Techniques

1. **Simplified collision geometry**: Use simpler shapes for collision detection
2. **Selective contact detection**: Disable contacts between non-critical bodies
3. **Multi-rate simulation**: Different time steps for different systems
4. **Level-of-detail**: Switch collision models based on distance

## Practical Exercise: Physics-Based Ball Rolling

Create a simulation scenario where:

1. A ball rolls down an inclined plane
2. The simulation accurately models:
   - Gravity (adjustable parameter)
   - Friction (adjustable parameter)
   - Collision with the plane
3. Measure and compare the simulated acceleration with the theoretical value

This exercise will help you understand how physics parameters affect simulation results.

## Troubleshooting Common Physics Issues

### Unstable Simulations

**Symptoms:** Objects jittering, exploding, or behaving erratically

**Solutions:**
- Reduce time step size
- Increase solver iterations
- Improve mass distribution in URDF
- Adjust constraint parameters (CFM, ERP)

### Penetration

**Symptoms:** Objects passing through each other

**Solutions:**
- Increase contact stiffness (kp)
- Improve collision geometry
- Use smaller time steps
- Enable continuous collision detection for fast objects

### Sliding Objects

**Symptoms:** Objects sliding when they should stay still

**Solutions:**
- Increase friction coefficients
- Adjust solver parameters
- Verify mass/inertia properties

## Summary

In this section, we've covered:

- The fundamental physics concepts essential for robot simulation
- How gravity, collisions, and friction are modeled in simulation
- Different physics engines and their characteristics
- Configuration options for Gazebo physics
- Performance considerations for realistic physics simulation
- Common troubleshooting approaches for physics-related issues

Understanding these physics principles is essential for creating realistic robot simulations that effectively bridge the gap between virtual and real-world robot behavior.