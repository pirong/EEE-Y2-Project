
using UnityEngine;
using UnityEngine.UI;
using NativeWebSocket;

public class AutonomousButton : MonoBehaviour
{
    public Button toggleButton;
    public Text buttonText;
    public GameObject joystick;
    public GameObject joystickBackground;
    public GameObject camera;

    public static bool isAutonomous = false;

    void Start()
    {
        toggleButton.onClick.AddListener(ToggleMode);
        UpdateUI();
    }

    void ToggleMode()
    {
        isAutonomous = !isAutonomous;
        UpdateUI();

        
        if (MotorControl.Instance != null && MotorControl.Instance.webSocket != null)
        {
            string modeMsg = isAutonomous ? "MODE:AUTONOMOUS" : "MODE:MANUAL";
            MotorControl.Instance.SendRawText(modeMsg);
        }
    }

    void UpdateUI()
    {
        if (isAutonomous)
        {
            buttonText.text = "Manual";
            joystick.SetActive(false);
            joystickBackground.SetActive(false);
            camera.SetActive(true);
        }
        else
        {
            buttonText.text = "Autonomous";
            joystick.SetActive(true);
            joystickBackground.SetActive(true);
            camera.SetActive(false);
        }
    }
}

