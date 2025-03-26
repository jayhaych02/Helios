using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class LidarSimulator : MonoBehaviour
{
    private ROSConnection ros;
    private string topicName = "/scout_robot/lidar";
    private float rangeMin = 0.1f;
    private float rangeMax = 100f;
    private float horizontalFov = 360f; // 360 degrees
    private int numSamples = 360; // 

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<LaserScanMsg>(topicName);
    }

    void Update()
    {
        float angleStep = horizontalFov / numSamples;
        float[] ranges = new float[numSamples];

        for (int i = 0; i < numSamples; i++)
        {
            float angle = i * angleStep * Mathf.Deg2Rad;
            Vector3 direction = new Vector3(Mathf.Cos(angle), 0, Mathf.Sin(angle));
            Ray ray = new Ray(transform.position, transform.TransformDirection(direction));
            RaycastHit hit;

            if (Physics.Raycast(ray, out hit, rangeMax))
            {
                ranges[i] = hit.distance;
            }
            else
            {
                ranges[i] = rangeMax;
            }
        }

        // Publish to ROS
        var laserScanMsg = new LaserScanMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg(),
            angle_min = 0f,
            angle_max = horizontalFov * Mathf.Deg2Rad,
            angle_increment = angleStep * Mathf.Deg2Rad,
            range_min = rangeMin,
            range_max = rangeMax,
            ranges = ranges
        };

        ros.Publish(topicName, laserScanMsg);
    }
}