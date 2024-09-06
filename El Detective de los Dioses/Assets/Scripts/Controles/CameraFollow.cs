using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    public Transform target; // Referencia al transform del personaje a seguir
    public float rotateSpeedX = 2.5f; // Velocidad de rotaci�n de la c�mara
    public float rotateSpeedY = 1f;

    private Vector3 offset = new Vector3(0f, 10f, -25f); // Desplazamiento de la c�mara con respecto al personaje
    private float currentAngleX = 0f; // �ngulo actual de rotaci�n en el eje X
    private float currentAngleY = 0f; // �ngulo actual de rotaci�n en el eje Y

    // Variables para el zoom
    public float zoomSpeed = 5f; // Velocidad del zoom
    public float minZoom = 5f; // M�nima distancia de zoom
    public float maxZoom = 50f; // M�xima distancia de zoom

    void LateUpdate()
    {
        // Rotaci�n de la c�mara alrededor del personaje basada en el movimiento del rat�n
        if (Input.GetMouseButton(0)) // Bot�n izquierdo del rat�n presionado
        {
            float mouseX = Input.GetAxis("Mouse X") * rotateSpeedX; // Obtener el movimiento horizontal del rat�n
            currentAngleX += mouseX; // Actualizar el �ngulo de rotaci�n en el eje X
            float mouseY = Input.GetAxis("Mouse Y") * rotateSpeedY;
            currentAngleY += mouseY;
            currentAngleY = Mathf.Clamp(currentAngleY, -30f, 45f);
        }

        // Ajustar el offset basado en la rueda del rat�n para el zoom
        float scroll = Input.GetAxis("Mouse ScrollWheel");
        float newMagnitude = Mathf.Clamp(offset.magnitude - scroll * zoomSpeed, minZoom, maxZoom);
        offset = offset.normalized * newMagnitude;

        // Aplicar la rotaci�n de la c�mara
        Quaternion rotation = Quaternion.Euler(-currentAngleY, currentAngleX, 0f); // Crear una rotaci�n basada en el �ngulo actual
        Vector3 newPosition = target.position + rotation * offset; // Calcular la nueva posici�n de la c�mara
        transform.position = newPosition; // Actualizar la posici�n de la c�mara
        transform.LookAt(target.position); // Mantener la c�mara mirando hacia el personaje en todo momento
    }
}

