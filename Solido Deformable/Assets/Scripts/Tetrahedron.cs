using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class Tetrahedron 
{
    public int id;

    // Vértices del tetraedro
    public Node nodo1;
    public Node nodo2;
    public Node nodo3;
    public Node nodo4;

    // Parámetros
    public float mass;
    public float volume;

    public Tetrahedron(int id, Node n1, Node n2, Node n3, Node n4, float densidad)
    {
        this.id = id;
        this.nodo1 = n1;
        this.nodo2 = n2;
        this.nodo3 = n3;
        this.nodo4 = n4;

        CalcularVolumen();
        CalcularMasa(densidad);
        AsignarMasaNodos();
    }

    private void CalcularVolumen()
    {
        // Para calcular el volumen del tetraedro, se realiza el producto mixto de sus vectores y se divide entre 6
        // V = ((r_2 - r_1) · (r_3 - r_1) x (r_4 - r_1)) / 6
        volume = Mathf.Abs(Vector3.Dot((nodo2.pos - nodo1.pos), Vector3.Cross((nodo3.pos - nodo1.pos), (nodo4.pos - nodo1.pos))))/6f;
    }

    private void CalcularMasa(float density)
    {
        // Se despeja la masa de la definición de densidad (masa/volumen)
        mass = volume * density;
    }

    private void AsignarMasaNodos()
    {
        // A cada nodo le corresponde una cuarta parte de la masa del tetraedro
        float masaNodos = mass / 4f;
        // Si un nodo pertenece a más de un tetraedro, se le acumula la masa
        nodo1.mass += masaNodos;
        nodo2.mass += masaNodos;
        nodo3.mass += masaNodos;
        nodo4.mass += masaNodos;
    }

    public bool Contains(Vector3 punto)
    {
        // Para calcular si el punto pertenece al tetraedro, se calcula el volumen de los 4 tetraedros que se forman con el punto a estudiar
        float volumenAcumulado = 0;
        // Para calcular el volumen del tetraedro, se realiza el producto mixto de sus vectores y se divide entre 6
        // V = ((r_2 - r_1) · (r_3 - r_1) x (r_4 - r_1)) / 6
        volumenAcumulado += Mathf.Abs(Vector3.Dot((nodo2.pos - punto), Vector3.Cross((nodo3.pos - punto), (nodo4.pos - punto)))) / 6f;
        volumenAcumulado += Mathf.Abs(Vector3.Dot((punto - nodo1.pos), Vector3.Cross((nodo3.pos - nodo1.pos), (nodo4.pos - nodo1.pos)))) / 6f;
        volumenAcumulado += Mathf.Abs(Vector3.Dot((nodo2.pos - nodo1.pos), Vector3.Cross((punto - nodo1.pos), (nodo4.pos - nodo1.pos)))) / 6f;
        volumenAcumulado += Mathf.Abs(Vector3.Dot((nodo2.pos - nodo1.pos), Vector3.Cross((nodo3.pos - nodo1.pos), punto - nodo1.pos))) / 6f;

        bool contiene = Mathf.Approximately(volumenAcumulado, volume); // Se comprueba si aproximadamente son iguales
        return contiene;
    }
}
