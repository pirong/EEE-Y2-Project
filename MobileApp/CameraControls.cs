using UnityEngine;

public class CameraControls : MonoBehaviour
{
    public Camera cam;
    public float panSpeed = 0.005f;
    public float zoomSpeed = 0.1f;
    public float rotationSpeed = 0.1f;
    public float minZoom = 3f;
    public float maxZoom = 20f;

    private Vector2 lastPanPosition;
    private bool isPanning;

    private float lastTouchDistance;
    private Vector2 lastTouch1Pos;
    private Vector2 lastTouch2Pos;

    void Update()
    {
        if (Input.touchCount == 1)
        {
            // One finger: Pan
            Touch touch = Input.GetTouch(0);

            if (touch.phase == TouchPhase.Began)
            {
                lastPanPosition = touch.position;
                isPanning = true;
            }
            else if (touch.phase == TouchPhase.Moved && isPanning)
            {
                Vector2 delta = touch.position - lastPanPosition;
                Vector3 move = new Vector3(-delta.x * panSpeed, -delta.y * panSpeed, 0);
                cam.transform.Translate(move, Space.Self);
                lastPanPosition = touch.position;
            }
            else if (touch.phase == TouchPhase.Ended)
            {
                isPanning = false;
            }
        }
        else if (Input.touchCount == 2)
        {
            // Two fingers: Zoom and Rotate
            Touch touch1 = Input.GetTouch(0);
            Touch touch2 = Input.GetTouch(1);

            // Calculate zoom
            Vector2 touch1Prev = touch1.position - touch1.deltaPosition;
            Vector2 touch2Prev = touch2.position - touch2.deltaPosition;

            float prevMagnitude = (touch1Prev - touch2Prev).magnitude;
            float currentMagnitude = (touch1.position - touch2.position).magnitude;

            float difference = currentMagnitude - prevMagnitude;

            ZoomCamera(-difference * zoomSpeed);

            // Calculate rotation
            Vector2 currentMidPoint = (touch1.position + touch2.position) * 0.5f;
            Vector2 previousMidPoint = (touch1Prev + touch2Prev) * 0.5f;

            Vector2 midPointDelta = currentMidPoint - previousMidPoint;

            // If fingers are moving roughly in the same direction (not pinching), rotate
            float angleDelta = (midPointDelta.y) * rotationSpeed;
            cam.transform.Rotate(-angleDelta,0f, 0f);
        }
    }

    void ZoomCamera(float increment)
    {
        if (cam.orthographic)
        {
            cam.orthographicSize = Mathf.Clamp(cam.orthographicSize + increment, minZoom, maxZoom);
        }
        else
        {
            cam.fieldOfView = Mathf.Clamp(cam.fieldOfView + increment, minZoom, maxZoom);
        }
    }
}
