using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable] // Se serializa la clase para que la lista de nodos sea visible desde el inspector
public class Node
{
    public float mass; // Masa del nodo
    public bool fixedNode; // Indica si el nodo es m�vil o fijo
    public Vector3 pos; // Vector posici�n del nodo
    public Vector3 vel; // Vector que almacena la direcci�n, el m�dulo y el sentido de la velocidad del nodo
    public Vector3 force; // Fuerza total que sufre el nodo

    public float dAbsolute; // Factor de amortiguamiento del nodo debido a su velocidad

    // Constructor que inicializa la masa y la posici�n de cada nodo al crearlo
    public Node(float m, Vector3 posicion, float dAbs)
    {
        mass = m; 
        pos = posicion;
        fixedNode = IsFixed();
        dAbsolute = dAbs;
    }

    // Funci�n encargada de calcular si un nodo es fijo o no. De esta forma, dependiendo del n�mero de los elementos fijadores o de su posici�n en la escena, habr� 
    // m�s o menos nodos fijos
    bool IsFixed()
    {
        // Primero se buscan todos los componentes fijadores en la escena, etiquetados con Fixer
        GameObject[] fixers = GameObject.FindGameObjectsWithTag("Fixer");
        // Despu�s, se recorre la lista de fixers
        foreach(GameObject fixer in fixers)
        {
            // Para cada uno, se comprueba si el nodo se encuentra dentro de los l�mites de su collider. Si es as�, la funci�n devuelve true
            // Se utilizan las coordenadas globales del nodo para calcularlo
            Collider fixerCollider = fixer.GetComponent<BoxCollider>();
            if(fixerCollider.bounds.Contains(pos))
            {
                Debug.Log("Nodo fijo");
                return true;
            }
        }
        // Si al recorrer todos los fixer, resulta que el nodo no se encuentra dentro de ninguno de ellos, quiere decir que no es fijo, por lo que se devuelve false
        return false;
    }


}
