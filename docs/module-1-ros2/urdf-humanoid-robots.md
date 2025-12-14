---
title: URDF for Humanoid Robots
sidebar_position: 4
---

# URDF for Humanoid Robots

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. URDF files define the physical and visual properties of a robot, including its links, joints, inertial properties, and visual appearance. For humanoid robots, URDF is particularly important as it describes the complex kinematic structure with multiple limbs and degrees of freedom.

## URDF Structure and Components

A URDF file is composed of several key elements that define different aspects of the robot:

### Links

Links represent rigid bodies in the robot. Each link has properties like mass, inertia, visual representation, and collision properties.

```xml
<link name="base_link">
  <inertial>
    <mass value="1.0" />
    <origin xyz="0 0 0" rpy="0 0 0" />
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01" />
  </inertial>
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <box size="0.2 0.2 0.2" />
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1" />
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <box size="0.2 0.2 0.2" />
    </geometry>
  </collision>
</link>
```

### Joints

Joints define the connections between links and specify the type of movement allowed between them.

```xml
<joint name="base_to_wheel" type="continuous">
  <parent link="base_link" />
  <child link="wheel_link" />
  <origin xyz="0.1 0.1 0" rpy="0 0 0" />
  <axis xyz="0 0 1" />
</joint>
```

## Joint Types in URDF

URDF supports several joint types that are essential for humanoid robot modeling:

### 1. Fixed Joint
A joint with no degrees of freedom (0 DOF).

```xml
<joint name="fixed_joint" type="fixed">
  <parent link="parent_link" />
  <child link="child_link" />
  <origin xyz="0 0 0.1" rpy="0 0 0" />
</joint>
```

### 2. Revolute Joint
A joint with one degree of freedom (1 DOF) that rotates around an axis, with limits.

```xml
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm" />
  <child link="forearm" />
  <origin xyz="0 0 -0.3" rpy="0 0 0" />
  <axis xyz="0 1 0" />
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1" />
</joint>
```

### 3. Continuous Joint
Like a revolute joint but with unlimited rotation (common for wheels).

```xml
<joint name="wheel_joint" type="continuous">
  <parent link="chassis" />
  <child link="wheel" />
  <origin xyz="0.2 0.2 0" rpy="0 0 0" />
  <axis xyz="0 0 1" />
</joint>
```

### 4. Prismatic Joint
A joint that slides along an axis (1 DOF translation).

```xml
<joint name="slider_joint" type="prismatic">
  <parent link="base" />
  <child link="slider" />
  <origin xyz="0 0 0" rpy="0 0 0" />
  <axis xyz="1 0 0" />
  <limit lower="0" upper="0.5" effort="100" velocity="1" />
</joint>
```

## Humanoid Robot URDF Example

Here's a simplified URDF for a humanoid robot with a torso, head, arms, and legs:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Torso -->
  <link name="torso">
    <inertial>
      <mass value="10.0" />
      <origin xyz="0 0 0.3" rpy="0 0 0" />
      <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.5" iyz="0.0" izz="0.2" />
    </inertial>
    <visual>
      <origin xyz="0 0 0.3" rpy="0 0 0" />
      <geometry>
        <box size="0.3 0.3 0.6" />
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.3" rpy="0 0 0" />
      <geometry>
        <box size="0.3 0.3 0.6" />
      </geometry>
    </collision>
  </link>

  <!-- Head -->
  <link name="head">
    <inertial>
      <mass value="2.0" />
      <origin xyz="0 0 0" rpy="0 0 0" />
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.05" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.15" />
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.15" />
      </geometry>
    </collision>
  </link>

  <!-- Neck joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso" />
    <child link="head" />
    <origin xyz="0 0 0.6" rpy="0 0 0" />
    <axis xyz="0 0 1" />
    <limit lower="-0.785" upper="0.785" effort="10" velocity="1" />
  </joint>

  <!-- Left upper arm -->
  <link name="left_upper_arm">
    <inertial>
      <mass value="1.5" />
      <origin xyz="0 0 -0.15" rpy="0 0 0" />
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.005" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.3" radius="0.05" />
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.3" radius="0.05" />
      </geometry>
    </collision>
  </link>

  <!-- Left shoulder joint -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso" />
    <child link="left_upper_arm" />
    <origin xyz="0.2 0 0.4" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1" />
  </joint>
</robot>
```

## URDF Tools and Visualization

### Robot State Publisher

The robot_state_publisher package takes URDF and joint positions and publishes the resulting transforms to tf2, which can be visualized in RViz.

### URDF in Gazebo

To use URDF with Gazebo, you need to add Gazebo-specific tags for physics simulation:

```xml
<gazebo reference="link_name">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
</gazebo>
```

### Transmission Elements

For controlling joints with ROS controllers, you need to define transmissions:

```xml
<transmission name="left_shoulder_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_shoulder_joint">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_shoulder_motor">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## Xacro: XML Macros for URDF

Xacro is an XML macro language that allows you to create more maintainable URDF files by using variables, macros, and includes:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_robot">

  <!-- Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="torso_height" value="0.6" />
  <xacro:property name="torso_radius" value="0.15" />

  <!-- Macro for arm -->
  <xacro:macro name="arm" params="side parent *origin">
    <joint name="${side}_shoulder_joint" type="revolute">
      <parent link="${parent}" />
      <child link="${side}_upper_arm" />
      <xacro:insert_block name="origin" />
      <axis xyz="0 1 0" />
      <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="20" velocity="1" />
    </joint>

    <link name="${side}_upper_arm">
      <inertial>
        <mass value="1.5" />
        <origin xyz="0 0 -0.15" rpy="0 0 0" />
        <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.005" />
      </inertial>
      <visual>
        <origin xyz="0 0 -0.15" rpy="0 0 0" />
        <geometry>
          <cylinder length="0.3" radius="0.05" />
        </geometry>
      </visual>
    </link>
  </xacro:macro>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <cylinder length="${torso_height}" radius="${torso_radius}" />
      </geometry>
    </visual>
  </link>

  <!-- Use the macro to create both arms -->
  <xacro:arm side="left" parent="torso">
    <origin xyz="${torso_radius} 0 ${torso_height/2}" rpy="0 0 0" />
  </xacro:arm>

  <xacro:arm side="right" parent="torso">
    <origin xyz="${-torso_radius} 0 ${torso_height/2}" rpy="0 0 0" />
  </xacro:arm>

</robot>
```

## Best Practices for Humanoid Robot URDF

### 1. Proper Inertial Properties
Accurate inertial properties are crucial for simulation and control:

- Calculate mass based on actual materials and dimensions
- Use CAD software to calculate accurate inertia tensors
- Verify that the center of mass is correctly positioned

### 2. Kinematic Chain Consistency
Ensure that joints form proper kinematic chains:

- Each link (except the base) should have exactly one parent
- Joint axes should be defined consistently
- Check for kinematic loops that can cause issues

### 3. Visual and Collision Separation
Use different geometries for visual and collision models:

- Visual models can be detailed for rendering
- Collision models should be simpler for performance
- Use multiple collision elements if needed for complex shapes

### 4. Naming Conventions
Use consistent naming conventions:

- Use descriptive names (e.g., "left_elbow_joint" instead of "j1")
- Follow a consistent pattern (side_component_type)
- Use underscores to separate words

## Practical Exercise: Create a Simple Humanoid Model

Create a URDF file for a simplified humanoid robot with:
1. A torso and head
2. Two arms with shoulder and elbow joints
3. Two legs with hip and knee joints
4. Proper inertial properties
5. Visual and collision elements

Use Xacro to make the model more maintainable by creating macros for repeated elements like arms and legs.

## Common Issues and Troubleshooting

### Self-Collision
Avoid self-collision by properly setting collision properties and using the `self_collide` tag when needed.

### Joint Limits
Set appropriate joint limits to prevent unrealistic poses that could damage a physical robot.

### Mass Distribution
Ensure mass is properly distributed to prevent simulation instability.

## Summary

In this section, we've covered:

- The structure and components of URDF files
- Different joint types and their applications
- How to create a humanoid robot model in URDF
- Tools for working with URDF (Xacro, robot_state_publisher)
- Best practices for humanoid robot modeling
- Common issues and how to address them

URDF is fundamental to robotics simulation and visualization, and mastering it is essential for working with humanoid robots in ROS 2.