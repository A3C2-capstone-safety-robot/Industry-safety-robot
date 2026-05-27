using UnityEngine;
using System;
using System.Collections;
using Unity.Collections;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;
using RosMessageTypes.Tf2;

public class CameraPublisher : MonoBehaviour
{
    [Header("카메라 설정")]
    public Camera robotCamera;
    public string imageTopic = "/camera/image_raw";
    public string cameraInfoTopic = "/camera/camera_info";
    public string tfTopic = "/tf";

    [Tooltip("해상도를 낮출수록 발행 속도가 빨라집니다. 320x240 권장.")]
    public int width = 320;
    public int height = 240;
    public float publishRate = 10f;
    public bool logPublishing = false;

    [Header("TF 프레임 이름")]
    public string mapFrame = "map";
    public string cameraFrame = "camera_frame";

    private ROSConnection ros;
    private RenderTexture rt;
    private Texture2D readTex;

    // 최적화용 변수들
    private byte[] rosData;
    private int rowBytes;
    private ImageMsg cachedImageMsg; // ImageMsg 재사용으로 GC 최소화

    void Start()
    {
        if (robotCamera == null)
        {
            Debug.LogError("[CameraPublisher] robotCamera가 할당되지 않았습니다.", this);
            enabled = false;
            return;
        }

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(imageTopic);
        ros.RegisterPublisher<CameraInfoMsg>(cameraInfoTopic);
        ros.RegisterPublisher<TFMessageMsg>(tfTopic);

        rt = new RenderTexture(width, height, 16, RenderTextureFormat.ARGB32);
        readTex = new Texture2D(width, height, TextureFormat.RGB24, false);
        robotCamera.targetTexture = rt;
        robotCamera.enabled = false;

        rowBytes = width * 3;
        rosData = new byte[width * height * 3];

        cachedImageMsg = new ImageMsg
        {
            header = new HeaderMsg { frame_id = cameraFrame },
            height = (uint)height,
            width = (uint)width,
            encoding = "rgb8",
            is_bigendian = 0,
            step = (uint)rowBytes,
            data = rosData
        };

        StartCoroutine(CaptureLoop());
        InvokeRepeating(nameof(PublishMeta), 1f, 1f / publishRate);
    }

    IEnumerator CaptureLoop()
    {
        float interval = 1f / publishRate;
        float next = Time.time + 1f;

        while (true)
        {
            yield return null;
            if (Time.time < next)
                continue;

            next = Time.time + interval;
            robotCamera.Render();

            RenderTexture previous = RenderTexture.active;
            RenderTexture.active = rt;
            readTex.ReadPixels(new Rect(0, 0, width, height), 0, 0);
            readTex.Apply();
            RenderTexture.active = previous;

            NativeArray<byte> raw = readTex.GetRawTextureData<byte>();

            for (int y = 0; y < height; y++)
            {
                int srcRow = (height - 1 - y) * rowBytes;
                int dstRow = y * rowBytes;
                NativeArray<byte>.Copy(raw, srcRow, rosData, dstRow, rowBytes);
            }

            cachedImageMsg.header.stamp = GetStamp();
            ros.Publish(imageTopic, cachedImageMsg);

            if (logPublishing)
                Debug.Log($"[CameraPublisher] Published image: {imageTopic}", this);
        }
    }

    void PublishMeta()
    {
        var stamp = GetStamp();
        PublishCameraInfo(stamp);
        PublishTF(stamp);
    }

    void PublishCameraInfo(RosMessageTypes.BuiltinInterfaces.TimeMsg stamp)
    {
        float fovRad = robotCamera.fieldOfView * Mathf.Deg2Rad;
        double f = (width / 2.0) / Math.Tan(fovRad / 2.0);

        ros.Publish(cameraInfoTopic, new CameraInfoMsg
        {
            header = new HeaderMsg { stamp = stamp, frame_id = cameraFrame },
            width = (uint)width,
            height = (uint)height,
            k = new double[9] { f, 0, width / 2.0, 0, f, height / 2.0, 0, 0, 1 },
            distortion_model = "plumb_bob",
            d = new double[5] { 0, 0, 0, 0, 0 }
        });
    }

    void PublishTF(RosMessageTypes.BuiltinInterfaces.TimeMsg stamp)
    {
        Vector3 uPos = robotCamera.transform.position;
        Quaternion uRot = robotCamera.transform.rotation;

        ros.Publish(tfTopic, new TFMessageMsg
        {
            transforms = new TransformStampedMsg[]
            {
                new TransformStampedMsg
                {
                    header         = new HeaderMsg { stamp = stamp, frame_id = mapFrame },
                    child_frame_id = cameraFrame,
                    transform      = new TransformMsg
                    {
                        translation = new Vector3Msg
                        {
                            x =  uPos.z,
                            y = -uPos.x,
                            z =  uPos.y
                        },
                        rotation = new QuaternionMsg
                        {
                            x = -uRot.z,
                            y =  uRot.x,
                            z = -uRot.y,
                            w =  uRot.w
                        }
                    }
                }
            }
        });
    }

    RosMessageTypes.BuiltinInterfaces.TimeMsg GetStamp()
    {
        long ms = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
        return new RosMessageTypes.BuiltinInterfaces.TimeMsg
        {
            sec = (int)(ms / 1000),
            nanosec = (uint)(ms % 1000 * 1_000_000)
        };
    }

    void OnDestroy()
    {
        if (rt != null) { rt.Release(); Destroy(rt); }
        if (readTex != null) Destroy(readTex);
    }
}
