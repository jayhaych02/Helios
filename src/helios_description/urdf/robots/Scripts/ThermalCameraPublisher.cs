using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class ThermalCameraPublisher : MonoBehaviour
{
    private Camera thermalCamera;
    private RenderTexture renderTexture;
    private Texture2D texture;
    private ROSConnection ros;
    private string topicName = "/firefighter_robot/thermal_camera";

    void Start()
    {
        thermalCamera = GetComponent<Camera>();
        renderTexture = thermalCamera.targetTexture;
        texture = new Texture2D(renderTexture.width, renderTexture.height, TextureFormat.RGB24, false);
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);
    }

    void Update()
    {
        // Capture the camera's render texture
        RenderTexture.active = renderTexture;
        texture.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0);
        texture.Apply();

        // Convert to ROS Image message
        byte[] imageData = texture.GetRawTextureData();
        var imageMsg = new ImageMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg(),
            height = (uint)renderTexture.height,
            width = (uint)renderTexture.width,
            encoding = "rgb8",
            is_bigendian = 0,
            step = (uint)(renderTexture.width * 3),
            data = imageData
        };

        // Publish to ROS
        ros.Publish(topicName, imageMsg);
    }
}