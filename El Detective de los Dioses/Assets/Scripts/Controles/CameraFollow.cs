using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    public Transform target; // Referencia al transform del personaje a seguir
    public float rotateSpeedX = 2.5f; // Velocidad de rotación de la cámara
    public float rotateSpeedY = 1f;

    private Vector3 offset = new Vector3(0f, 10f, -25f); // Desplazamiento de la cámara con respecto al personaje
    private float currentAngleX = 0f; // Ángulo actual de rotación en el eje X
    private float currentAngleY = 0f; // Ángulo actual de rotación en el eje Y

    // Variables para el zoom
    public float zoomSpeed = 5f; // Velocidad del zoom
    public float minZoom = 5f; // Mínima distancia de zoom
    public float maxZoom = 50f; // Máxima distancia de zoom

    void LateUpdate()
    {
        // Rotación de la cámara alrededor del personaje basada en el movimiento del ratón
        if (Input.GetMouseButton(0)) // Botón izquierdo del ratón presionado
        {
            float mouseX = Input.GetAxis("Mouse X") * rotateSpeedX; // Obtener el movimiento horizontal del ratón
            currentAngleX += mouseX; // Actualizar el ángulo de rotación en el eje X
            float mouseY = Input.GetAxis("Mouse Y") * rotateSpeedY;
            currentAngleY += mouseY;
            currentAngleY = Mathf.Clamp(currentAngleY, -30f, 45f);
        }

        // Ajustar el offset basado en la rueda del ratón para el zoom
        float scroll = Input.GetAxis("Mouse ScrollWheel");
        float newMagnitude = Mathf.Clamp(offset.magnitude - scroll * zoomSpeed, minZoom, maxZoom);
        offset = offset.normalized * newMagnitude;

        // Aplicar la rotación de la cámara
        Quaternion rotation = Quaternion.Euler(-currentAngleY, currentAngleX, 0f); // Crear una rotación basada en el ángulo actual
        Vector3 newPosition = target.position + rotation * offset; // Calcular la nueva posición de la cámara
        transform.position = newPosition; // Actualizar la posición de la cámara
        transform.LookAt(target.position); // Mantener la cámara mirando hacia el personaje en todo momento
    }
}

