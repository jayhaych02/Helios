using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class IMUSimulator : MonoBehaviour
{
    private ArticulationBody articulationBody;
    private ROSConnection ros;
    private string topicName = "/firefighter_robot/imu";
    private Vector3 lastVelocity;

    void Start()
    {
        articulationBody = GetComponentInParent<ArticulationBody>();
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImuMsg>(topicName);
        lastVelocity = Vector3.zero;
    }

    void FixedUpdate()
    {
        // Calculate acceleration
        Vector3 velocity = articulationBody.velocity;
        Vector3 acceleration = (velocity - lastVelocity) / Time.fixedDeltaTime;
        lastVelocity = velocity;

        // Get orientation
        Quaternion orientation = transform.rotation;

        // Publish to ROS
        var imuMsg = new ImuMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg(),
            orientation = new RosMessageTypes.Geometry.QuaternionMsg(orientation.x, orientation.y, orientation.z, orientation.w),
            linear_acceleration = new RosMessageTypes.Geometry.Vector3Msg(acceleration.x, acceleration.y, acceleration.z)
        };

        ros.Publish(topicName, imuMsg);
    }
}