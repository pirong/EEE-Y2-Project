using UnityEngine;
using NativeWebSocket;
using Newtonsoft.Json;
using System.Collections.Generic;

public class SLAMMapReceiver : MonoBehaviour
{
    WebSocket websocket;

    public GameObject robotObject;
    public float mapResolution = 0.5f;
    public Vector2 mapOrigin = new Vector2(-5, -5); 

    [System.Serializable]
    public class RobotPose
    {
        public float x;
        public float y;
        public float theta;
    }

    [System.Serializable]
    public class SLAMMap
    {
        public int width;
        public int height;
        public List<int> data;
        public RobotPose robot_pose;
    }

    async void Start()
    {
        websocket = new WebSocket("ws://10.198.203.24:8765");

        websocket.OnOpen += () => Debug.Log("WebSocket connected.Slam");
        websocket.OnError += (e) => Debug.LogError($"WebSocket error: {e}");
        websocket.OnClose += (e) => Debug.Log("WebSocket closed.Slam");

        websocket.OnMessage += (bytes) =>
        {
            string json = System.Text.Encoding.UTF8.GetString(bytes);
            SLAMMap map = JsonConvert.DeserializeObject<SLAMMap>(json);
            DrawTexture(map);
        };

        await websocket.Connect();
    }

    void Update()
    {
#if !UNITY_WEBGL || UNITY_EDITOR
        websocket?.DispatchMessageQueue();
#endif
    }

    async void OnApplicationQuit()
    {
        await websocket.Close();
    }

    public async void ReconnectWebSocket(string ip)
    {
        if (websocket != null)
        {
            await websocket.Close();
        }

        websocket = new WebSocket($"ws://{ip}:8765");

        PlayerPrefs.SetString("baseURL2", ip);
        PlayerPrefs.Save();

        websocket.OnOpen += () => Debug.Log("SLAM WebSocket connected.");
        websocket.OnError += (e) => Debug.LogError($"WebSocket error: {e}");
        websocket.OnClose += (e) => Debug.Log("WebSocket closed.");

        websocket.OnMessage += (bytes) =>
        {
            string json = System.Text.Encoding.UTF8.GetString(bytes);
            SLAMMap map = JsonConvert.DeserializeObject<SLAMMap>(json);
            DrawTexture(map);
        };

        await websocket.Connect();
    }

    void DrawTexture(SLAMMap map)
    {
        Texture2D texture = new Texture2D(map.width, map.height);
        for (int y = 0; y < map.height; y++)
        {
            for (int x = 0; x < map.width; x++)
            {
                int i = y * map.width + x;
                int value = map.data[i];

                Color color;

                if (value == -1)
                {
                    color = Color.magenta;
                }
                else
                {
                    float normalized = 1f - (value / 100f);
                    color = new Color(normalized, normalized, normalized);
                }

                texture.SetPixel(x, y, color);
            }
        }

        texture.Apply();
        GetComponent<Renderer>().material.mainTexture = texture;

        
        float mapWidthMeters = map.width * mapResolution;
        float mapHeightMeters = map.height * mapResolution;

        
        transform.localScale = new Vector3(mapWidthMeters / 10f, 1f, mapHeightMeters / 10f);

        // position robot on the map
        if (robotObject != null && map.robot_pose != null)
        {
            float x = map.robot_pose.x;
            float y = map.robot_pose.y;
            float theta = map.robot_pose.theta;

            float unityX = (x - mapOrigin.x) / mapResolution;
            float unityZ = (y - mapOrigin.y) / mapResolution;

            robotObject.transform.position = new Vector3(-unityX, robotObject.transform.position.y, -unityZ);
            robotObject.transform.rotation = Quaternion.Euler(90, 90 - theta * Mathf.Rad2Deg, 0);
        }
    }
}
