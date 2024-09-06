using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable] // Se serializa la clase para que la lista de nodos sea visible desde el inspector
public class Node
{
    public float mass; // Masa del nodo
    public bool fixedNode; // Indica si el nodo es móvil o fijo
    public Vector3 pos; // Vector posición del nodo
    public Vector3 vel; // Vector que almacena la dirección, el módulo y el sentido de la velocidad del nodo
    public Vector3 force; // Fuerza total que sufre el nodo

    public float dAbsolute; // Factor de amortiguamiento del nodo debido a su velocidad

    // Constructor que inicializa la masa y la posición de cada nodo al crearlo
    public Node(float m, Vector3 posicion, float dAbs)
    {
        mass = m; 
        pos = posicion;
        fixedNode = IsFixed();
        dAbsolute = dAbs;
    }

    // Función encargada de calcular si un nodo es fijo o no. De esta forma, dependiendo del número de los elementos fijadores o de su posición en la escena, habrá 
    // más o menos nodos fijos
    bool IsFixed()
    {
        // Primero se buscan todos los componentes fijadores en la escena, etiquetados con Fixer
        GameObject[] fixers = GameObject.FindGameObjectsWithTag("Fixer");
        // Después, se recorre la lista de fixers
        foreach(GameObject fixer in fixers)
        {
            // Para cada uno, se comprueba si el nodo se encuentra dentro de los límites de su collider. Si es así, la función devuelve true
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
