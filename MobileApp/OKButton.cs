using UnityEngine;
using UnityEngine.UI;
using NativeWebSocket;

public class OKButton : MonoBehaviour
{
    public Text inputText;
    public MotorControl motorControl; // Assign this in the Inspector!

    public Text inputText2;
    public SLAMMapReceiver slammapReceiver; // Assign this in the Inspector!

   

    public void OkClicked()
    {
        string newIP = inputText.text;
        string newIP2 = inputText2.text;

        if (motorControl != null && AutonomousButton.isAutonomous == false)
        {
            motorControl.ReconnectWebSocket(newIP);
        }
        else if(slammapReceiver != null && AutonomousButton.isAutonomous == true)
        {
            slammapReceiver.ReconnectWebSocket(newIP2);
        }
        else
        {
            Debug.LogError("no change");
        }
    }
}







