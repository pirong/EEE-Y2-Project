using UnityEngine;
using UnityEngine.UI;

public class JoystickMotorControl : MonoBehaviour
{
    public MotorControl motorControl;

    public FixedJoystick variableJoystick;

    public Text text;

    public void Update()
    {
        //variableJoystick.Vertical gives a range between from -1 to 1 and is varied by the position of the slider.
        int sliderValue = Mathf.RoundToInt((variableJoystick.Vertical)*2);

        
    }
}


