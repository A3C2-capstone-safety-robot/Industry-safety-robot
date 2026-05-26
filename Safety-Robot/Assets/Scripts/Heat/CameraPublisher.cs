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
        // 디버그 로그 추가 1
        //Debug.Log("[CameraPublisher] Start() 진입 성공!");
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(imageTopic);
        ros.RegisterPublisher<CameraInfoMsg>(cameraInfoTopic);
        ros.RegisterPublisher<TFMessageMsg>(tfTopic);

        // 오버플로우 방지를 위해 ARGB32 / RGBA32(픽셀당 4바이트)로 통일
        rt = new RenderTexture(width, height, 16, RenderTextureFormat.ARGB32);
        readTex = new Texture2D(width, height, TextureFormat.RGBA32, false);
        robotCamera.targetTexture = rt;

        rowBytes = width * 4; // 픽셀당 4바이트 (RGBA)
        rosData = new byte[width * height * 4];

        // 1회성 고정 데이터 미리 세팅
        cachedImageMsg = new ImageMsg
        {
            header = new HeaderMsg { frame_id = cameraFrame },
            height = (uint)height,
            width = (uint)width,
            encoding = "rgba8", // 인코딩을 rgba8로 변경
            is_bigendian = 0,
            step = (uint)rowBytes,
            data = rosData
        };

        StartCoroutine(CaptureLoop());
        InvokeRepeating(nameof(PublishMeta), 1f, 1f / publishRate);

        // 디버그 로그 추가 2
        //Debug.Log("[CameraPublisher] Start() 셋업 완료 및 코루틴 시작!");
    }

    IEnumerator CaptureLoop()
    {
        var eof = new WaitForEndOfFrame();
        float interval = 1f / publishRate;
        float next = Time.time + 1f;

        // 디버그 로그 추가 3
        //Debug.Log("[CameraPublisher] CaptureLoop 코루틴 정상 진입!");

        while (true)
        {
            yield return eof;

            if (Time.time < next) continue;
            next = Time.time + interval;

            // 디버그 로그 추가 4 (이 로그가 콘솔에 계속 주기적으로 찍혀야 합니다)
            //Debug.Log($"[CameraPublisher] 이미지 발행 중... 시간: {Time.time}");

            // ── 텍스처 읽기 ──────────────────────────────────────────
            RenderTexture.active = rt;
            readTex.ReadPixels(new Rect(0, 0, width, height), 0, 0);
            readTex.Apply();
            RenderTexture.active = null;

            // ── Y축 반전 행 단위 고속 복사 ───────────────────────────
            NativeArray<byte> raw = readTex.GetRawTextureData<byte>();

            for (int y = 0; y < height; y++)
            {
                int srcRow = (height - 1 - y) * rowBytes;
                int dstRow = y * rowBytes;
                NativeArray<byte>.Copy(raw, srcRow, rosData, dstRow, rowBytes);
            }

            // 타임스탬프만 변경 후 발행 (가비지 프리)
            cachedImageMsg.header.stamp = GetStamp();
            //Debug.Log($"[CameraPublisher] ros.Publish 호출: {imageTopic}");
            ros.Publish(imageTopic, cachedImageMsg);
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
        return new RosMessageTypes.BuiltinInterfaces.TimeMsg
        {
            sec = (int)Time.time,
            nanosec = (uint)((Time.time - (int)Time.time) * 1e9f)
        };
    }

    void OnDestroy()
    {
        if (rt != null) { rt.Release(); Destroy(rt); }
        if (readTex != null) Destroy(readTex);
    }
}