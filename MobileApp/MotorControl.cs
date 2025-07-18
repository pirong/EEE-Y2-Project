using UnityEngine;
using NativeWebSocket;
using UnityEngine.UI;

public class MotorControl : MonoBehaviour
{
    public DynamicJoystick joystick;
    public static string baseURL = "192.168.254.187";

    public WebSocket webSocket;
    private float sendInterval = 0.05f;
    private float lastSendTime = 0f;

    public static MotorControl Instance;

    public Text Status;

    public Text vbatText;
    public Text v5Text;
    public Text current5VText;
    public Text currentMotorText;

    public Text power5VText;
    public Text powerMotorText;
    public Text powerBatteryText;

    public Text currentBatteryText;
    public Text vmotorText;

    public Text batteryPercentText;
    public Text batteryEnergyText;

    public bool isConnected = false;

    [System.Serializable]
    public class TelemetryMessage
    {
        public float vbat;
        public float v5;
        public float current_5v;
        public float current_motor;
    }

    private void Awake()
    {
        Instance = this;
        baseURL = PlayerPrefs.GetString("baseURL", "192.168.254.187");
        ConnectWebSocket(baseURL);
        Status.text = "Status: Not Connected";
    }

    async void ConnectWebSocket(string ip)
    {
        webSocket = new WebSocket($"ws://{ip}:81");

        webSocket.OnOpen += () =>
        {
            Debug.Log("WebSocket connected.");
            isConnected = true;
        };

        webSocket.OnError += (e) =>
        {
            Debug.LogError($"WebSocket Error: {e}");
            isConnected = false;
        };

        webSocket.OnClose += (e) =>
        {
            Debug.Log("WebSocket closed.");
            isConnected = false;
        };

        webSocket.OnMessage += (bytes) =>
        {
            string msg = System.Text.Encoding.UTF8.GetString(bytes);
            Debug.Log($"Message from ESP32: {msg}");
            try
            {
                var telemetry = JsonUtility.FromJson<TelemetryMessage>(msg);

                float vbat = telemetry.vbat;
                float v5 = telemetry.v5;
             
                float i5 = telemetry.current_5v;
                float imotor = telemetry.current_motor;
               

                float power5V = v5 * i5;
                float powerMotor = vbat * imotor;
                float powerBattery = power5V + powerMotor;

                float ibat = powerBattery / vbat;

                float percent = estimateBatteryPercent(vbat);
                float remainingEnergy = vbat * 2.0f * (percent / 100.0f);

                // Display
                vbatText.text = $"{vbat:F2} V";
                vmotorText.text = $"{vbat:F2} V";
                v5Text.text = $"{v5:F3} V";
                current5VText.text = $"{i5:F3} A";
                currentMotorText.text = $"{imotor:F3} A";
                currentBatteryText.text = $"{ibat:F3} A";

                power5VText.text = $"{power5V:F2} W";
                powerMotorText.text = $"{powerMotor:F2} W";
                powerBatteryText.text = $"{powerBattery:F2} W";

                batteryPercentText.text = $"{percent:F1} %";
                batteryEnergyText.text = $"{remainingEnergy:F2} Wh";
            }
            catch
            {
                Debug.LogWarning("Received non-telemetry message.");
            }
        };

        await webSocket.Connect();
    }

    private void Update()
    {
        if (webSocket != null)
            webSocket.DispatchMessageQueue();

       
        if (!isConnected)
        {
            ShowZeroTelemetry();
            Status.text = "Status: Not Connected";
        }
        else
        {
            if (AutonomousButton.isAutonomous)
            {
                Status.text = "Status: Finding Suspicious Person";
            }
            else
            {
                Status.text = "Status: Manual Control Connected";
            }
        }
        
        if (AutonomousButton.isAutonomous) return;

        if (Time.time - lastSendTime >= sendInterval)
        {
            lastSendTime = Time.time;

            float x = -joystick.Horizontal;
            float y = joystick.Vertical;

            SendXY(x, y);
        }
    }
    
    public async void SendRawText(string msg)
    {
        if (webSocket != null && webSocket.State == WebSocketState.Open)
        {
            await webSocket.SendText(msg);
        }
    }
    async void SendXY(float x, float y)
    {
        if (webSocket != null && webSocket.State == WebSocketState.Open)
        {
            string json = JsonUtility.ToJson(new XYMessage(x, y));
            await webSocket.SendText(json);
        }
    }

    public async void ReconnectWebSocket(string newIP)
    {
        baseURL = newIP;
        PlayerPrefs.SetString("baseURL", newIP);
        PlayerPrefs.Save();

        if (webSocket != null)
        {
            await webSocket.Close();
        }

        ConnectWebSocket(newIP);
    }

    private async void OnApplicationQuit()
    {
        if (webSocket != null)
        {
            await webSocket.Close();
        }
    }

    private void ShowZeroTelemetry()
    {
        vbatText.text = "0.00 V";
        vmotorText.text = "0.00 V";
        v5Text.text = "0.000 V";
        current5VText.text = "0.000 A";
        currentMotorText.text = "0.000 A";
        currentBatteryText.text = "0.000 A";

        power5VText.text = "0.00 W";
        powerMotorText.text = "0.00 W";
        powerBatteryText.text = "0.00 W";

        batteryPercentText.text = "0.0 %";
        batteryEnergyText.text = "0.00 Wh";
    }

    [System.Serializable]
    public class XYMessage
    {
        public float x;
        public float y;

        public XYMessage(float xVal, float yVal)
        {
            x = xVal;
            y = yVal;
        }
    }

    // --- Battery percentage lookup ---

    const int tableSize = 37;
    float[] voltageTable = {
        16.08f,15.65f,15.37f,15.17f,15.03f,14.93f,14.842f,14.778f,14.732f,14.685f,
        14.635f,14.602f,14.569f,14.526f,14.498f,14.472f,14.445f,14.417f,14.391f,14.366f,
        14.341f,14.311f,14.283f,14.252f,14.22f,14.185f,14.149f,14.11f,14.068f,14.024f,
        13.974f,13.912f,13.851f,13.78f,13.698f,13.595f,13.462f
    };
    float[] percentTable = {
        100f,97.22f,94.44f,91.67f,88.89f,86.11f,83.33f,80.56f,77.78f,75f,
        72.22f,69.44f,66.67f,63.89f,61.11f,58.33f,55.56f,52.78f,50f,47.22f,
        44.44f,41.67f,38.89f,36.11f,33.33f,30.56f,27.78f,25f,22.22f,19.44f,
        16.67f,13.89f,11.11f,8.33f,5.56f,2.78f,0f
    };

    float estimateBatteryPercent(float vbat)
    {
        if (vbat >= voltageTable[0]) return 100f;
        if (vbat <= voltageTable[tableSize - 1]) return 0f;

        for (int i = 0; i < tableSize - 1; i++)
        {
            if (vbat < voltageTable[i] && vbat > voltageTable[i + 1])
            {
                float v1 = voltageTable[i];
                float v2 = voltageTable[i + 1];
                float p1 = percentTable[i];
                float p2 = percentTable[i + 1];

                float percent = p1 + ((vbat - v1) / (v2 - v1)) * (p2 - p1);
                return percent;
            }
        }
        return 0f;
    }
}

