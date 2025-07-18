using UnityEngine;
using UnityEngine.UI;

public class InputText2 : MonoBehaviour
{
    public InputField inputField;

    private void Start()
    {
        //sets the input text field to the stored baseURL value when the app is launched
        inputField.text = PlayerPrefs.GetString("baseURL2", "");


    }
}





