using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class Spring
{
    public float k; // Constante de rigidez del muelle, que dependerá de si es un muelle de tracción o de flexión
    public float l0; // Longitud inicial del muelle en reposo
    public float l; // Longitud instantánea del muelle
    public Vector3 dir; // Vector unitario de su dirección

    public Node nodeA; // Extremo A del muelle
    public Node nodeB; // Extremo B del muelle

    public float dDeformation; // Factor de amortiguamiento del muelle

    public Spring (float k, Node nA, Node nB, float dDef)
    {
        this.k = k;
        this.nodeA = nA;
        this.nodeB = nB;
        // La longitud inicial del muelle es la distancia entre sus extremos
        this.l0 = (nA.pos - nB.pos).magnitude;
        this.l = l0; // En el inicio, la longitud instantánea es igual a la longitud inicial
        this.dir = (nA.pos - nB.pos).normalized; // Vector director normalizado
        this.dDeformation = dDef;
    }

}
