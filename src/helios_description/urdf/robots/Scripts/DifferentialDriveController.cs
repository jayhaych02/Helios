using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class DifferentialDriveController : MonoBehaviour
{
    // ROS Connection
    private ROSConnection ros;

    // Articulation Bodies for the wheels
    public ArticulationBody leftWheel;
    public ArticulationBody rightWheel;

    // Wheel parameters (from URDF)
    public float wheelSeparation = 0.4f; // Distance between wheels (from URDF: wheel_separation=0.4)
    public float wheelRadius = 0.05f;    // Wheel radius (from URDF: wheel_radius=0.05)

    // Speed parameters
    private float linearSpeed = 0f;
    private float angularSpeed = 0f;

    void Awake()
    {
        Debug.Log("DifferentialDriveController Awake called for " + gameObject.name);
    }

    void Start()
    {
        // Get the ROS connection instance
        ros = ROSConnection.GetOrCreateInstance();
        // Subscribe to the /cmd_vel topic
        ros.Subscribe<TwistMsg>("/cmd_vel", CmdVelCallback);
        Debug.Log("Subscribed to /cmd_vel topic for " + gameObject.name);
        Debug.Log($"ROS Connection State: {ros.HasConnectionError}");
    }

    void Update()
    {
        Debug.Log($"ROS Connection State (Update): {ros.HasConnectionError} for {gameObject.name}");
    }

    void CmdVelCallback(TwistMsg msg)
    {
        Debug.Log($"Received /cmd_vel message: linear.x={msg.linear.x}, angular.z={msg.angular.z} for {gameObject.name}");
        // Extract linear and angular velocities from the message
        linearSpeed = (float)msg.linear.x;
        angularSpeed = (float)msg.angular.z;
    }

    void FixedUpdate()
    {
        // Convert linear and angular velocities to wheel velocities
        float leftWheelSpeed = (linearSpeed - angularSpeed * wheelSeparation / 2) / wheelRadius;
        float rightWheelSpeed = (linearSpeed + angularSpeed * wheelSeparation / 2) / wheelRadius;

        // Convert to radians per second (ArticulationBody expects angular velocity in rad/s)
        leftWheelSpeed *= Mathf.Rad2Deg;  // Convert to degrees per second (Unity's ArticulationBody uses degrees)
        rightWheelSpeed *= Mathf.Rad2Deg;

        // Apply velocities to the wheels
        if (leftWheel != null)
        {
            var leftDrive = leftWheel.xDrive;
            leftDrive.targetVelocity = leftWheelSpeed;
            leftWheel.xDrive = leftDrive;
        }

        if (rightWheel != null)
        {
            var rightDrive = rightWheel.xDrive;
            rightDrive.targetVelocity = rightWheelSpeed;
            rightWheel.xDrive = rightDrive;
        }
    }
}