using System.Collections;
using System.Collections.Generic;
using UnityEditor.Experimental.GraphView;
using UnityEditor;
using UnityEngine;
[System.Serializable]
public class Vertex
{
    public Vector3 posicionVertice; // Posición del vértice
    public Tetrahedron tetraedroContenedor; // Tetraedro que contiene el punto 
    // Coordenadas baricéntricas
    public float w1;
    public float w2;
    public float w3;
    public float w4;

    public Vertex(Vector3 posicionVertice, Tetrahedron tetraedroContenedor)
    {
        this.posicionVertice = posicionVertice; // Posición del vértice de la malla
        this.tetraedroContenedor = tetraedroContenedor; // Tetraedro contenedor
    }

    public void CalcularCoordenadasBaricentricas()
    {
        // Se dividen los volúmenes de los tetraedros que se forman tomando el vértice entre el volumen total
        // Coordenada baricéntrica 1 (Vértices P, 2, 3 y 4)
        float Vi = Mathf.Abs(Vector3.Dot((tetraedroContenedor.nodo2.pos - posicionVertice), Vector3.Cross((tetraedroContenedor.nodo3.pos - posicionVertice), (tetraedroContenedor.nodo4.pos - posicionVertice)))) / 6f;
        w1 = Vi / tetraedroContenedor.volume;

        // Coordenada baricéntrica 2 (Vértices 1, P, 3 y 4)
        Vi = Mathf.Abs(Vector3.Dot((posicionVertice - tetraedroContenedor.nodo1.pos), Vector3.Cross((tetraedroContenedor.nodo3.pos - tetraedroContenedor.nodo1.pos), (tetraedroContenedor.nodo4.pos - tetraedroContenedor.nodo1.pos)))) / 6f;
        w2 = Vi / tetraedroContenedor.volume;

        // Coordenada baricéntrica 3 (Vértices 1, 2, P y 4)
        Vi = Mathf.Abs(Vector3.Dot((tetraedroContenedor.nodo2.pos - tetraedroContenedor.nodo1.pos), Vector3.Cross((posicionVertice - tetraedroContenedor.nodo1.pos), (tetraedroContenedor.nodo4.pos - tetraedroContenedor.nodo1.pos)))) / 6f;
        w3 = Vi / tetraedroContenedor.volume;

        // Coordenada baricéntrica 4 (Vértices 1, 2, 3 y P)
        Vi = Mathf.Abs(Vector3.Dot((tetraedroContenedor.nodo2.pos - tetraedroContenedor.nodo1.pos), Vector3.Cross((tetraedroContenedor.nodo3.pos - tetraedroContenedor.nodo1.pos), posicionVertice - tetraedroContenedor.nodo1.pos))) / 6f;
        w4 = Vi / tetraedroContenedor.volume;
    }

    public void ActualizarPosicion()
    {
        // Para actualizar la posición del vértice, se interpolan los valores de los nodos del tetraedro aplicando las coordenadas baricéntricas calculadas
        posicionVertice = tetraedroContenedor.nodo1.pos * w1 +
                            tetraedroContenedor.nodo2.pos * w2 +
                            tetraedroContenedor.nodo3.pos * w3 +
                            tetraedroContenedor.nodo4.pos * w4;
    }
}
