using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Reflection;
using UnityEditor.Experimental.GraphView;
using UnityEngine;
using UnityEngine.UIElements;

public class MassSpringSolid : MonoBehaviour
{
    // Referencia a la malla
    Mesh malla;

    // Ficheros de texto con la informaci�n del .stl obtenida mediante TetGen
    [SerializeField] TextAsset ficheroNodos;
    [SerializeField] TextAsset ficheroTetraedros;

    // Lista de nodos
    [SerializeField] List<Node> listaNodos;
    // Lista de tetraedros
    [SerializeField] List<Tetrahedron> listaTetraedros;
    // Estructura para guardar los v�rtices de las aristas
    [System.Serializable]
    public struct Edge: IComparable<Edge>
    {
        public int vA, vB;
        public float volumen; // Esta variable se utiliza para despu�s crear los muelles. A cada arista le correspone 1/6 del volumen total del tetraedro
        // Constructor
        public Edge(int vA, int vB, float v)
        {
            this.vA = vA;
            this.vB = vB;
            this.volumen = v;
        }
        // Se implementa la interfaz comparable, para poder ordenar una lista de aristas en el futuro, y as� identificar aristas duplicadas
        public int CompareTo(Edge other)
        {
            // Primer se compara el v�rtice A
            if (vA != other.vA)
                return vA.CompareTo(other.vA);
            // Si son iguales, se compara el v�rtice B
            return vB.CompareTo(other.vB);
        }
    }

    public enum Integration
    {
        ExplicitEuler = 0,
        SimplecticEuler = 1
    }

    public Integration integrationMethod; // Variable que en el inspector selecciona el m�todo de integraci�n num�rica a realizar

    // Lista de aristas
    [SerializeField] List<Edge> aristas;
    // Lista de muelles
    [SerializeField] List<Spring> listaMuelles;
    // Lista de v�rtices
    [SerializeField] List<Vertex> listaVertices;

    // PAR�METROS
    [SerializeField] float dAbs = 0.1f; // Factor de amortiguamiento de los nodos
    [SerializeField] float p = 0.01f; // Densidad del objeto, que se utiliza para calcular la masa de los tetraedros
    [SerializeField] float k = 100f; // Constante de rigidez de los muelles
    [SerializeField] float dDef = 1.0f; // Factor de amortiguamiento de los muelles
    [SerializeField] float h = 0.01f; // Paso del tiempo
    [SerializeField] Vector3 g = new Vector3(0.0f, 9.81f, 0.0f); // Vector con el valor de la gravedad

    // Start is called before the first frame update
    void Start()
    {
        malla = GetComponent<MeshFilter>().mesh;
        CultureInfo locale = new CultureInfo("en-US"); // Se indica el idioma para realizar correctamente la conversi�n de string a float (ya que los decimales est�n con ".")
        // Primero se agregan los nodos a la lista, con la informaci�n del fichero
        AgregarNodos(ficheroNodos, locale);
        // Despu�s, se crean los tetraedros de la misma forma
        CrearTetraedros(ficheroTetraedros);
        // Utilizando la informaci�n de los tetraedros, se crean las aristas
        GenerarAristas();
        // Una vez hechas las aristas, se generan los muelles
        CrearMuelles();
        // Por �ltimo, se halla el tetraedro contenedor de cada v�rtice y sus coordenadas baric�ntricas
        EstablecerVertices();
    }

    // Update is called once per frame
    void Update()
    {
        malla.MarkDynamic();
        Vector3[] newVertex = malla.vertices;

        // La transformaci�n se aplica al v�rtice de la malla, seg�n la transformaci�n calculada al interpolar la posici�n de los nodos
        for (int i = 0; i < newVertex.Length; i++)
        {
            newVertex[i] = transform.InverseTransformPoint(listaVertices[i].posicionVertice);
        }
        malla.MarkDynamic();
        malla.vertices = newVertex;
    }

    private void FixedUpdate()
    {
        // En funci�n del m�todo de integraci�n escogido, se utiliza una funci�n u otra
        switch (integrationMethod)
        {
            case Integration.ExplicitEuler:
                integrationExplicitEuler();
                break;
            case Integration.SimplecticEuler:
                integrationSimplecticEuler();
                break;
            default:
                print("ERROR METODO INTEGRACION DESCONOCIDO");
                break;
        }

        // Una vez hechos los c�lculos con el m�todo de integraci�n, se actualizan los par�metros de los muelles seg�n los resultados obtenidos
        foreach (Spring muelle in listaMuelles)
        {
            // Direcci�n
            muelle.dir = (muelle.nodeA.pos - muelle.nodeB.pos).normalized;
            // Nueva longitud
            muelle.l = (muelle.nodeA.pos - muelle.nodeB.pos).magnitude;
        }

        // Despu�s, se actualizan las posiciones de los v�rtices, tras calcular la nueva posici�n de los nodos
        foreach (Vertex verticeMalla in listaVertices)
        {
            verticeMalla.ActualizarPosicion();
        }
    }

    void integrationExplicitEuler()
    {
        // Primero se recorre la lista de nodos para aplicarles las fuerzas individuales que sufren (peso y amortiguamiento)
        foreach (Node n in listaNodos)
        {
            // Reinicio de la fuerza
            n.force = Vector3.zero;

            // Si el nodo es fijo, no se actualiza su posici�n
            if (n.fixedNode)
            {
                continue;
            }
            // 1 - Actualizaci�n de su posici�n con la velocidad del paso anterior
            n.pos += h * n.vel;

            // 2 - C�lculo de fuerzas
            n.force += (g * n.mass) - (n.dAbsolute * n.vel * n.mass); // Peso (P = m * g) y amortiguamiento (F = - d * m * v)

        }
        // Despu�s, se recorre la lista de muelles, para aplicarles a todos los nodos las fuerzas el�sticas y de amortiguamiento debido a estos
        foreach (Spring muelle in listaMuelles)
        {
            // Fuerza el�stica -> F = - k * (L-L0) * d -> Tiene sentidos opuestos para los dos extremos del muele
            muelle.nodeA.force += -muelle.k * (muelle.l - muelle.l0) * muelle.dir;
            muelle.nodeB.force += muelle.k * (muelle.l - muelle.l0) * muelle.dir;

            // Fuerza de amortiguamiento debida a a la velocidad relativa entre los dos nodos del muelle
            applyDampingSpring(muelle);
        }
        // 3 - Una vez calculadas todas las fuerzas de cada nodo, se actualiza su velocidad
        foreach (Node n in listaNodos)
        {
            if (n.fixedNode)
            {
                continue;
            }
            n.vel += h * (n.force / n.mass); // Seg�n la 2� Ley de Newton , F = m * a -> a = F / m 
        }
    }

    void integrationSimplecticEuler()
    {
        // Primero se recorre la lista de nodos para aplicarles las fuerzas individuales que sufren (peso y amortiguamiento)
        foreach (Node n in listaNodos)
        {
            // 1 - C�lculo de fuerzas 
            // Reinicio de la fuerza
            n.force = Vector3.zero;
            // C�lculo de la fuerza sobre un nodo
            n.force += (g * n.mass) - (n.dAbsolute * n.vel * n.mass); // Peso (P = m * g) y amortiguamiento (F = - d * m * v)
        }
        // Despu�s, se recorre la lista de muelles, para aplicarles a todos los nodos las fuerzas el�sticas y de amortiguamiento debido a estos

        foreach (Spring muelle in listaMuelles)
        {
            // Fuerza el�stica
            applyElasticForce(muelle);

            // Fuerza de amortiguamiento debida a a la velocidad relativa entre los dos nodos del muelle
            applyDampingSpring(muelle);
        }


        foreach (Node n in listaNodos)
        {
            // Si el nodo es fijo, no se actualizan su posici�n y velocidad
            if (n.fixedNode)
            {
                continue;
            }
            // 2. Actualizaci�n de su velocidad
            n.vel += h * (n.force / n.mass); // Seg�n la 2� Ley de Newton , F = m * a -> a = F / m 
            // 3. Actualizaci�n de su posici�n con la velocidad del paso actual
            n.pos += h * n.vel;
        }
    }

    private string[] LeerFichero(TextAsset fichero)
    {
        // Se pasa del fichero de texto a un array, que cuenta con el n�mero de nodos y las posiciones de estos
        string[] texto = fichero.text.Split(new string[] { " ", "\n", "\r" }, System.StringSplitOptions.RemoveEmptyEntries);
        return texto;
    }

    private void AgregarNodos(TextAsset fichero, CultureInfo locale)
    {
        string[] textoNodos = LeerFichero(fichero);
        // Se accede a las posiciones de los v�rtices mediante el array obtenido
        int numNodos = int.Parse(textoNodos[0]); // N�mero de nodos (v�rtices de la malla)
        int numCoordenadas = int.Parse(textoNodos[1]); // N�mero de coordenadas por v�rtice
        int numCoordenadasNodos = numNodos * (numCoordenadas + 1); // Total de elementos que se deben recorrer
        // Se recorren las posiciones de los nodos -> SE INVIERTEN LOS EJES YZ
        for(int i = 4; i <= numCoordenadasNodos; i+= (numCoordenadas + 1))
        {
            int id = int.Parse(textoNodos[i]) - 1; // Se toma su identificador
            Vector3 posicionNodo = new Vector3(float.Parse(textoNodos[i + 1], locale), float.Parse(textoNodos[i + 3], locale), float.Parse(textoNodos[i + 2], locale)); // Se crea la posici�n del nodo, inviertiendo YZ
            Node nodo = new Node(id, transform.TransformPoint(posicionNodo), dAbs); // Creaci�n del nodo, en coordenadas globales. Al identificador se le resta uno porque tetgen comienza en 1
            listaNodos.Add(nodo);
        }
    }

    private void CrearTetraedros(TextAsset fichero)
    {
        string[] textoTetraedros = LeerFichero(fichero);
        // Se acceden a los �ndices de los nodos de los tetraedros mediante el array obtenido
        int numTetraedros = int.Parse(textoTetraedros[0]); // Total de tetraedros que tiene la malla del envolvente
        int numVertices = int.Parse(textoTetraedros[1]); // N�mero de v�rtices de cada tetraedro (4)
        int numNodosTetraedros = numTetraedros * (numVertices + 1); // Total de elementos que se deben recorrer
        // Se recorren los �ndices de los nodos que componen los v�rtices de cada tetraedro
        for(int i= 3; i<= numNodosTetraedros; i+= (numVertices + 1))
        {
            int id = int.Parse(textoTetraedros[i]) - 1;
            // Obtenci�n de los nodos de los v�rtices, previamente almacenados en la lista
            Node nodo1 = listaNodos[int.Parse(textoTetraedros[i + 1]) - 1]; // Se le resta 1 al �ndice del nodo obtenido, ya que se empieza a partir de 1 en lugar de 0 como en la lista
            Node nodo2 = listaNodos[int.Parse(textoTetraedros[i + 2]) - 1];
            Node nodo3 = listaNodos[int.Parse(textoTetraedros[i + 3]) - 1];
            Node nodo4 = listaNodos[int.Parse(textoTetraedros[i + 4]) - 1];
            // Creaci�n del tetraedro
            Tetrahedron tetraedro = new Tetrahedron(id, nodo1, nodo2, nodo3, nodo4, p);
            // Se almacena en la lista de tetraedros
            listaTetraedros.Add(tetraedro);
        }
    }

    private void GenerarAristas()
    {
        // Para crear las aristas, se recorre la lista de tetraedros y se emparejan sus v�rtices formando 6 combinaciones en total (los tres lados de la base m�s los que unen con el v�rtice superior)
        for(int i = 0; i < listaTetraedros.Count; i++)
        {
            float volumenArista = listaTetraedros[i].volume / 6f;
            // Se ordenan sus v�rtices de menor a mayor, para facilitar la eliminaci�n de aristas duplicadas
            // Arista 1 - V1 y V2
            Edge arista1;
            if (listaTetraedros[i].nodo1.id_nodo <= listaTetraedros[i].nodo2.id_nodo)
            {
                arista1 = new Edge(listaTetraedros[i].nodo1.id_nodo, listaTetraedros[i].nodo2.id_nodo, volumenArista);
            }
            else
            {
                arista1 = new Edge(listaTetraedros[i].nodo2.id_nodo, listaTetraedros[i].nodo1.id_nodo, volumenArista);
            }
            // Arista 2 - V1 y V3
            Edge arista2;
            if (listaTetraedros[i].nodo1.id_nodo <= listaTetraedros[i].nodo3.id_nodo)
            {
                arista2 = new Edge(listaTetraedros[i].nodo1.id_nodo, listaTetraedros[i].nodo3.id_nodo, volumenArista);
            }
            else
            {
                arista2 = new Edge(listaTetraedros[i].nodo3.id_nodo, listaTetraedros[i].nodo1.id_nodo, volumenArista);
            }
            // Arista 3 - V2 y V3
            Edge arista3;
            if (listaTetraedros[i].nodo2.id_nodo <= listaTetraedros[i].nodo3.id_nodo)
            {
                arista3 = new Edge(listaTetraedros[i].nodo2.id_nodo, listaTetraedros[i].nodo3.id_nodo, volumenArista);
            }
            else
            {
                arista3 = new Edge(listaTetraedros[i].nodo3.id_nodo, listaTetraedros[i].nodo2.id_nodo, volumenArista);
            }
            // Arista 4 - V1 y V4
            Edge arista4;
            if (listaTetraedros[i].nodo1.id_nodo <= listaTetraedros[i].nodo4.id_nodo)
            {
                arista4 = new Edge(listaTetraedros[i].nodo1.id_nodo, listaTetraedros[i].nodo4.id_nodo, volumenArista);
            }
            else
            {
                arista4 = new Edge(listaTetraedros[i].nodo4.id_nodo, listaTetraedros[i].nodo1.id_nodo, volumenArista);
            }
            // Arista 5 - V2 y V4
            Edge arista5;
            if (listaTetraedros[i].nodo2.id_nodo <= listaTetraedros[i].nodo4.id_nodo)
            {
                arista5 = new Edge(listaTetraedros[i].nodo2.id_nodo, listaTetraedros[i].nodo4.id_nodo, volumenArista);
            }
            else
            {
                arista5 = new Edge(listaTetraedros[i].nodo4.id_nodo, listaTetraedros[i].nodo2.id_nodo, volumenArista);
            }
            // Arista 6 - V3 y V4
            Edge arista6;
            if (listaTetraedros[i].nodo3.id_nodo <= listaTetraedros[i].nodo4.id_nodo)
            {
                arista6 = new Edge(listaTetraedros[i].nodo3.id_nodo, listaTetraedros[i].nodo4.id_nodo, volumenArista);
            }
            else
            {
                arista6 = new Edge(listaTetraedros[i].nodo4.id_nodo, listaTetraedros[i].nodo3.id_nodo, volumenArista);
            }

            // Se a�aden las aristas a la lista
            aristas.Add(arista1);
            aristas.Add(arista2);
            aristas.Add(arista3);
            aristas.Add(arista4);
            aristas.Add(arista5);
            aristas.Add(arista6);
        }
        // Una vez creadas y a�adidas a la lista, esta se ordena para detectar aristas duplicadas
        aristas.Sort();
    }

    private void CrearMuelles()
    {
        // Para crear los muelles, se debe recorrer la lista de aristas, evitando que haya m�s de un muelle para la misma arista. Sin embargo, debe almacenar el volumen acumulado
        // de las aristas duplicadas
        // Variable auxiliar para ir almacenando el volumen acumulado
        float volumenAcumulado = 0;
        for(int i = 0; i < aristas.Count - 1; i++)
        {
            // Si los extremos de la arista coinciden con la arista siguiente, no se crea un muelle, s�lo se acumula el volumen
            if ((aristas[i].vA == aristas[i+1].vA) && (aristas[i].vB == aristas[i+1].vB))
            {
                volumenAcumulado += aristas[i].volumen;
            }
            else
            {
                volumenAcumulado += aristas[i].volumen;
                Spring muelle = new Spring(k, listaNodos[aristas[i].vA], listaNodos[aristas[i].vB], dDef, volumenAcumulado);
                volumenAcumulado = 0; // Se reinicia el contador del volumen acumulado
                listaMuelles.Add(muelle); // Se a�ade el muelle a la lista de muelles
            }
        }
        // Para evitar rebasar los l�mites de la lista, el �ltimo muelle se crea de forma manual
        Spring muelleFinal = new Spring(k, listaNodos[aristas[aristas.Count - 1].vA], listaNodos[aristas[aristas.Count - 1].vB], dDef, aristas[aristas.Count - 1].volumen);
        listaMuelles.Add(muelleFinal);
    }

    private void EstablecerVertices()
    {
        // Se obtienen los v�rtices de la malla (posiciones)
        Vector3[] posicionesVertices = malla.vertices;
        // Primero se identifica el tetraedro contenedor de cada v�rtice
        for(int i = 0; i < posicionesVertices.Length; i++)
        {
            // Para ello, se recorre la lista de tetraedros
            for(int j = 0; j < listaTetraedros.Count; j++)
            {
                if (listaTetraedros[j].Contains(posicionesVertices[i]))
                {
                    // Primero se crea el v�rtice junto con su tetraedro asociado
                    Vertex vertice = new Vertex(posicionesVertices[i], listaTetraedros[j]);
                    // Una vez hecho esto, se establecen sus coordenadas baric�ntricas
                    vertice.CalcularCoordenadasBaricentricas();
                    listaVertices.Add(vertice);
                    break;
                }
            }
        }
    }

    // Funci�n para calcular la nueva fuerza el�stica seg�n la densidad de los muelles
    private void applyElasticForce(Spring muelle)
    {
        // Ambos nodos de cada muelle sufren la misma fuerza, con sentidos opuestos
        // F = (-V / L0^2) * k * (L - L0) * (rA - rB) / L0
        muelle.nodeA.force += (-muelle.volume / (muelle.l0 * muelle.l0)) * muelle.k * (muelle.l - muelle.l0) * ((muelle.nodeA.pos - muelle.nodeB.pos) / muelle.l0);
        muelle.nodeB.force += -(-muelle.volume / (muelle.l0 * muelle.l0)) * muelle.k * (muelle.l - muelle.l0) * ((muelle.nodeA.pos - muelle.nodeB.pos) / muelle.l0);
    }

    // Funci�n para aplicar el amortiguamiento en los muelles
    private void applyDampingSpring(Spring muelle)
    {
        // Ambos nodos de cada muelle sufren la misma fuerza, con sentidos opuestos
        // F = - d * [(vA - VB) * u)] * u
        muelle.nodeA.force += -muelle.dDeformation * Vector3.Dot((muelle.nodeA.vel - muelle.nodeB.vel), muelle.dir) * muelle.dir;
        muelle.nodeB.force += muelle.dDeformation * Vector3.Dot((muelle.nodeA.vel - muelle.nodeB.vel), muelle.dir) * muelle.dir;
    }

    private void OnDrawGizmos()
    {
        // Dibujo de los nodos
        Gizmos.color = Color.yellow;
        foreach (Node node in listaNodos)
        {
            Gizmos.DrawSphere(node.pos, 0.4f);
        }
        // Dibujo de los muelles
        Gizmos.color = Color.blue;
        foreach (Spring spring in listaMuelles)
        {
            Gizmos.DrawLine(spring.nodeA.pos, spring.nodeB.pos);
        }
    }
}
