---
title: Unity Rendering for Human-Robot Interaction (HRI)
sidebar_position: 3
---

# Unity Rendering for Human-Robot Interaction (HRI)

## Introduction to Unity for Robotics

Unity is a powerful game engine that has found significant applications in robotics, particularly for Human-Robot Interaction (HRI) scenarios. Unlike physics-focused simulators like Gazebo, Unity excels at creating visually realistic environments and intuitive interfaces for human-robot interaction. Unity's real-time rendering capabilities make it ideal for creating immersive HRI experiences.

## Unity in the Robotics Ecosystem

Unity has become increasingly important in robotics through initiatives like:

- **Unity Robotics Hub**: Provides tools and assets for robotics simulation
- **Unity ML-Agents**: For training AI agents in Unity environments
- **ROS# (ROS Sharp)**: .NET-based ROS bridge for Unity
- **Unity Perception**: Tools for generating synthetic training data

## Setting Up Unity for Robotics

### Unity Installation and Configuration

1. **Install Unity Hub**: Download from Unity's website
2. **Install Unity Editor**: Choose LTS (Long Term Support) version
3. **Install Robotics Packages**: Through Unity's Package Manager
4. **Configure ROS Bridge**: Set up communication with ROS/ROS2

### Required Packages

```json
{
  "dependencies": {
    "com.unity.robotics.ros-tcp-connector": "0.7.0",
    "com.unity.robotics.urdf-importer": "0.5.2",
    "com.unity.robotics.visualizations": "0.1.0"
  }
}
```

## Unity Scene Structure for Robotics

### Basic Scene Setup

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class RobotSceneManager : MonoBehaviour
{
    void Start()
    {
        // Initialize ROS connection
        ROSConnection.instance = gameObject.AddComponent<ROSConnection>();
        ROSConnection.instance.rosIPAddress = "127.0.0.1";
        ROSConnection.instance.rosPort = 10000;
    }
}
```

### Coordinate System Conversion

Unity uses a left-handed coordinate system (X-right, Y-up, Z-forward), while ROS uses a right-handed system (X-forward, Y-left, Z-up). Proper conversion is essential:

```csharp
public static Vector3 Ros2Unity(Vector3 rosPosition)
{
    return new Vector3(rosPosition.y, rosPosition.z, rosPosition.x);
}

public static Vector3 Unity2Ros(Vector3 unityPosition)
{
    return new Vector3(unityPosition.z, unityPosition.x, unityPosition.y);
}
```

## Human-Robot Interaction in Unity

### Visual Feedback Systems

Unity excels at providing rich visual feedback for HRI:

#### 1. Attention Indicators
```csharp
public class AttentionIndicator : MonoBehaviour
{
    public GameObject attentionLight;
    public Material activeMaterial;
    public Material inactiveMaterial;

    public void SetAttention(bool isActive)
    {
        attentionLight.GetComponent<Renderer>().material =
            isActive ? activeMaterial : inactiveMaterial;
    }
}
```

#### 2. Gesture Visualization
```csharp
public class GestureVisualizer : MonoBehaviour
{
    public LineRenderer gestureTrail;
    public List<Vector3> gesturePoints;

    void UpdateGestureTrail()
    {
        gestureTrail.positionCount = gesturePoints.Count;
        gestureTrail.SetPositions(gesturePoints.ToArray());
    }
}
```

### Interactive Interfaces

#### Touch and Gesture Recognition
```csharp
using UnityEngine;

public class HRIInterface : MonoBehaviour
{
    void Update()
    {
        if (Input.touchCount > 0)
        {
            Touch touch = Input.GetTouch(0);
            if (touch.phase == TouchPhase.Began)
            {
                Ray ray = Camera.main.ScreenPointToRay(touch.position);
                RaycastHit hit;

                if (Physics.Raycast(ray, out hit))
                {
                    ProcessTouchInteraction(hit.collider.gameObject);
                }
            }
        }
    }

    void ProcessTouchInteraction(GameObject touchedObject)
    {
        // Handle HRI interaction
        if (touchedObject.CompareTag("RobotControl"))
        {
            SendCommandToRobot(touchedObject.name);
        }
    }
}
```

#### Voice Command Integration
```csharp
using UnityEngine;
using System.Collections;

public class VoiceCommandHandler : MonoBehaviour
{
    [System.Serializable]
    public class CommandMapping
    {
        public string command;
        public string rosTopic;
        public string message;
    }

    public List<CommandMapping> commandMappings;

    public void ProcessVoiceCommand(string command)
    {
        foreach (var mapping in commandMappings)
        {
            if (command.ToLower().Contains(mapping.command.ToLower()))
            {
                // Send ROS message
                PublishToROS(mapping.rosTopic, mapping.message);
                return;
            }
        }
    }

    void PublishToROS(string topic, string message)
    {
        // Implementation for sending ROS messages
    }
}
```

## Unity for Robot Visualization

### URDF Import and Animation

Unity's URDF Importer allows importing robot models from ROS:

```csharp
using Unity.Robotics.UrdfImporter;

public class RobotController : MonoBehaviour
{
    public GameObject robotModel;
    private JointControl[] joints;

    void Start()
    {
        // Find all joint controllers in the robot model
        joints = robotModel.GetComponentsInChildren<JointControl>();
    }

    public void SetJointPositions(float[] positions)
    {
        for (int i = 0; i < joints.Length && i < positions.Length; i++)
        {
            joints[i].SetJointPosition(positions[i]);
        }
    }
}
```

### Real-time Robot State Visualization

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class RobotStateVisualizer : MonoBehaviour
{
    public GameObject robotModel;
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.instance;
        ros.Subscribe<JointStateMsg>("joint_states", OnJointStateReceived);
    }

    void OnJointStateReceived(JointStateMsg jointState)
    {
        // Update robot visualization based on joint states
        UpdateRobotJoints(jointState.name, jointState.position);
    }

    void UpdateRobotJoints(string[] jointNames, float[] jointPositions)
    {
        // Map ROS joint states to Unity robot model
        for (int i = 0; i < jointNames.Length; i++)
        {
            Transform jointTransform = FindJointByName(jointNames[i]);
            if (jointTransform != null)
            {
                // Apply joint position to Unity transform
                ApplyJointPosition(jointTransform, jointPositions[i]);
            }
        }
    }

    Transform FindJointByName(string jointName)
    {
        // Implementation to find joint by name in robot hierarchy
        return robotModel.transform.Find(jointName);
    }

    void ApplyJointPosition(Transform joint, float position)
    {
        // Apply the position to the joint's rotation or position
        joint.localEulerAngles = new Vector3(0, position * Mathf.Rad2Deg, 0);
    }
}
```

## Creating HRI Scenarios

### Social Robot Behaviors

Unity can simulate social robot behaviors for HRI research:

```csharp
using UnityEngine;

public class SocialRobotBehavior : MonoBehaviour
{
    public enum RobotState
    {
        Idle,
        Attending,
        Responding,
        Active
    }

    public RobotState currentState = RobotState.Idle;
    public float attentionRadius = 2.0f;
    public Transform robotHead;

    void Update()
    {
        UpdateRobotState();
        HandleSocialBehaviors();
    }

    void UpdateRobotState()
    {
        // Detect nearby humans using colliders or raycasting
        Collider[] nearbyHumans = Physics.OverlapSphere(
            transform.position, attentionRadius);

        if (nearbyHumans.Length > 0)
        {
            currentState = RobotState.Attending;
            LookAtNearestHuman();
        }
        else
        {
            currentState = RobotState.Idle;
        }
    }

    void LookAtNearestHuman()
    {
        // Find nearest human and look at them
        Collider nearest = GetNearestCollider();
        if (nearest != null)
        {
            Vector3 lookPos = nearest.transform.position;
            lookPos.y = robotHead.position.y; // Keep head level
            robotHead.LookAt(lookPos);
        }
    }

    Collider GetNearestCollider()
    {
        // Implementation to find nearest collider
        return Physics.OverlapSphere(transform.position, attentionRadius)
            .OrderBy(c => Vector3.Distance(c.transform.position, transform.position))
            .FirstOrDefault();
    }

    void HandleSocialBehaviors()
    {
        switch (currentState)
        {
            case RobotState.Attending:
                ShowAttentionVisuals();
                break;
            case RobotState.Responding:
                PlayResponseAnimation();
                break;
        }
    }

    void ShowAttentionVisuals()
    {
        // Activate attention indicators
    }

    void PlayResponseAnimation()
    {
        // Play appropriate response animation
    }
}
```

### Multi-Modal Interaction

Creating rich HRI experiences with multiple interaction modalities:

```csharp
using UnityEngine;
using UnityEngine.UI;

public class MultiModalHRI : MonoBehaviour
{
    public Text statusText;
    public Image attentionIndicator;
    public AudioSource audioSource;
    public Animator robotAnimator;

    public void HandleUserInput(string inputType, object inputData)
    {
        switch (inputType)
        {
            case "voice":
                ProcessVoiceInput((string)inputData);
                break;
            case "gesture":
                ProcessGestureInput((Vector3)inputData);
                break;
            case "touch":
                ProcessTouchInput((string)inputData);
                break;
        }
    }

    void ProcessVoiceInput(string command)
    {
        statusText.text = $"Heard: {command}";
        RespondToVoiceCommand(command);
    }

    void ProcessGestureInput(Vector3 gestureVector)
    {
        statusText.text = "Gesture detected";
        RespondToGesture(gestureVector);
    }

    void ProcessTouchInput(string touchTarget)
    {
        statusText.text = $"Touched: {touchTarget}";
        RespondToTouch(touchTarget);
    }

    void RespondToVoiceCommand(string command)
    {
        // Play response animation and audio
        robotAnimator.SetTrigger("Listen");
        PlayResponseAudio("I heard you say: " + command);
    }

    void RespondToGesture(Vector3 gesture)
    {
        robotAnimator.SetTrigger("GestureResponse");
    }

    void RespondToTouch(string target)
    {
        robotAnimator.SetTrigger("TouchResponse");
    }

    void PlayResponseAudio(string message)
    {
        // Implementation for text-to-speech or audio playback
    }
}
```

## Performance Optimization for Real-time HRI

### Rendering Optimization

For smooth HRI experiences:

```csharp
using UnityEngine;

public class HRIPerformanceOptimizer : MonoBehaviour
{
    public int targetFrameRate = 60;
    public bool enableLOD = true;
    public float lodDistance = 10.0f;

    void Start()
    {
        Application.targetFrameRate = targetFrameRate;
        QualitySettings.vSyncCount = 0; // Disable vsync for consistent frame rate
    }

    void Update()
    {
        if (enableLOD)
        {
            UpdateLevelOfDetail();
        }
    }

    void UpdateLevelOfDetail()
    {
        // Simplify geometry based on distance from camera
        float distance = Vector3.Distance(Camera.main.transform.position, transform.position);
        bool useSimpleGeometry = distance > lodDistance;

        // Switch between detailed and simplified models
        SetGeometryComplexity(useSimpleGeometry);
    }

    void SetGeometryComplexity(bool simple)
    {
        // Implementation to switch between detailed/simplified geometry
    }
}
```

### Network Optimization for ROS Communication

```csharp
using System.Collections;
using Unity.Robotics.ROSTCPConnector;

public class OptimizedROSCommunication : MonoBehaviour
{
    private ROSConnection ros;
    public float publishRate = 30.0f; // Hz
    private float lastPublishTime;

    void Start()
    {
        ros = ROSConnection.instance;
        lastPublishTime = Time.time;
    }

    void Update()
    {
        if (Time.time - lastPublishTime >= 1.0f / publishRate)
        {
            PublishRobotState();
            lastPublishTime = Time.time;
        }
    }

    void PublishRobotState()
    {
        // Publish only necessary data at required frequency
        // Avoid publishing high-frequency data that isn't needed
    }
}
```

## Practical Exercise: Interactive Robot Assistant

Create a Unity scene with:

1. A robot model that responds to voice commands
2. Visual feedback when the robot is listening
3. Simple animations for different robot states
4. Touch interaction to trigger robot behaviors
5. Integration with ROS for actual robot control

This exercise will help you understand how to create engaging HRI experiences in Unity.

## Unity vs. Gazebo for Different Use Cases

### When to Use Unity

- **Visual realism** is more important than physics accuracy
- **Human-robot interaction** research and design
- **Training scenarios** requiring high visual fidelity
- **Public demonstrations** where visual appeal matters
- **Mixed reality** applications combining virtual and real elements

### When to Use Gazebo

- **Physics accuracy** is critical for control development
- **Sensor simulation** with realistic noise models
- **Multi-robot coordination** with accurate dynamics
- **Control algorithm** development and validation
- **Performance testing** under realistic physical constraints

## Integration with ROS/ROS2

### ROS-TCP-Connector

Unity provides a TCP connector for ROS communication:

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class UnityROSIntegration : MonoBehaviour
{
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.instance;

        // Subscribe to ROS topics
        ros.Subscribe<StringMsg>("unity_status", OnUnityStatusReceived);

        // Publish to ROS topics
        InvokeRepeating("PublishUnityData", 0, 1.0f); // Every second
    }

    void OnUnityStatusReceived(StringMsg status)
    {
        Debug.Log("Received from ROS: " + status.data);
    }

    void PublishUnityData()
    {
        var message = new StringMsg();
        message.data = "Unity simulation running";
        ros.Publish("unity_status", message);
    }
}
```

## Summary

In this section, we've covered:

- How Unity differs from physics-focused simulators for robotics
- Setting up Unity for robotics applications and HRI
- Creating rich visual feedback systems for HRI
- Implementing multi-modal interaction in Unity
- Optimizing Unity for real-time HRI applications
- When to use Unity vs. other simulation platforms
- Integration techniques with ROS/ROS2

Unity's strength in visual rendering and user interaction makes it an excellent choice for HRI research and applications where visual fidelity and intuitive interfaces are important.