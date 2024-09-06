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

    // Ficheros de texto con la información del .stl obtenida mediante TetGen
    [SerializeField] TextAsset ficheroNodos;
    [SerializeField] TextAsset ficheroTetraedros;

    // Lista de nodos
    [SerializeField] List<Node> listaNodos;
    // Lista de tetraedros
    [SerializeField] List<Tetrahedron> listaTetraedros;
    // Estructura para guardar los vértices de las aristas
    [System.Serializable]
    public struct Edge: IComparable<Edge>
    {
        public int vA, vB;
        public float volumen; // Esta variable se utiliza para después crear los muelles. A cada arista le correspone 1/6 del volumen total del tetraedro
        // Constructor
        public Edge(int vA, int vB, float v)
        {
            this.vA = vA;
            this.vB = vB;
            this.volumen = v;
        }
        // Se implementa la interfaz comparable, para poder ordenar una lista de aristas en el futuro, y así identificar aristas duplicadas
        public int CompareTo(Edge other)
        {
            // Primer se compara el vértice A
            if (vA != other.vA)
                return vA.CompareTo(other.vA);
            // Si son iguales, se compara el vértice B
            return vB.CompareTo(other.vB);
        }
    }

    public enum Integration
    {
        ExplicitEuler = 0,
        SimplecticEuler = 1
    }

    public Integration integrationMethod; // Variable que en el inspector selecciona el método de integración numérica a realizar

    // Lista de aristas
    [SerializeField] List<Edge> aristas;
    // Lista de muelles
    [SerializeField] List<Spring> listaMuelles;
    // Lista de vértices
    [SerializeField] List<Vertex> listaVertices;

    // PARÁMETROS
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
        CultureInfo locale = new CultureInfo("en-US"); // Se indica el idioma para realizar correctamente la conversión de string a float (ya que los decimales están con ".")
        // Primero se agregan los nodos a la lista, con la información del fichero
        AgregarNodos(ficheroNodos, locale);
        // Después, se crean los tetraedros de la misma forma
        CrearTetraedros(ficheroTetraedros);
        // Utilizando la información de los tetraedros, se crean las aristas
        GenerarAristas();
        // Una vez hechas las aristas, se generan los muelles
        CrearMuelles();
        // Por último, se halla el tetraedro contenedor de cada vértice y sus coordenadas baricéntricas
        EstablecerVertices();
    }

    // Update is called once per frame
    void Update()
    {
        malla.MarkDynamic();
        Vector3[] newVertex = malla.vertices;

        // La transformación se aplica al vértice de la malla, según la transformación calculada al interpolar la posición de los nodos
        for (int i = 0; i < newVertex.Length; i++)
        {
            newVertex[i] = transform.InverseTransformPoint(listaVertices[i].posicionVertice);
        }
        malla.MarkDynamic();
        malla.vertices = newVertex;
    }

    private void FixedUpdate()
    {
        // En función del método de integración escogido, se utiliza una función u otra
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

        // Una vez hechos los cálculos con el método de integración, se actualizan los parámetros de los muelles según los resultados obtenidos
        foreach (Spring muelle in listaMuelles)
        {
            // Dirección
            muelle.dir = (muelle.nodeA.pos - muelle.nodeB.pos).normalized;
            // Nueva longitud
            muelle.l = (muelle.nodeA.pos - muelle.nodeB.pos).magnitude;
        }

        // Después, se actualizan las posiciones de los vértices, tras calcular la nueva posición de los nodos
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

            // Si el nodo es fijo, no se actualiza su posición
            if (n.fixedNode)
            {
                continue;
            }
            // 1 - Actualización de su posición con la velocidad del paso anterior
            n.pos += h * n.vel;

            // 2 - Cálculo de fuerzas
            n.force += (g * n.mass) - (n.dAbsolute * n.vel * n.mass); // Peso (P = m * g) y amortiguamiento (F = - d * m * v)

        }
        // Después, se recorre la lista de muelles, para aplicarles a todos los nodos las fuerzas elásticas y de amortiguamiento debido a estos
        foreach (Spring muelle in listaMuelles)
        {
            // Fuerza elástica -> F = - k * (L-L0) * d -> Tiene sentidos opuestos para los dos extremos del muele
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
            n.vel += h * (n.force / n.mass); // Según la 2ª Ley de Newton , F = m * a -> a = F / m 
        }
    }

    void integrationSimplecticEuler()
    {
        // Primero se recorre la lista de nodos para aplicarles las fuerzas individuales que sufren (peso y amortiguamiento)
        foreach (Node n in listaNodos)
        {
            // 1 - Cálculo de fuerzas 
            // Reinicio de la fuerza
            n.force = Vector3.zero;
            // Cálculo de la fuerza sobre un nodo
            n.force += (g * n.mass) - (n.dAbsolute * n.vel * n.mass); // Peso (P = m * g) y amortiguamiento (F = - d * m * v)
        }
        // Después, se recorre la lista de muelles, para aplicarles a todos los nodos las fuerzas elásticas y de amortiguamiento debido a estos

        foreach (Spring muelle in listaMuelles)
        {
            // Fuerza elástica
            applyElasticForce(muelle);

            // Fuerza de amortiguamiento debida a a la velocidad relativa entre los dos nodos del muelle
            applyDampingSpring(muelle);
        }


        foreach (Node n in listaNodos)
        {
            // Si el nodo es fijo, no se actualizan su posición y velocidad
            if (n.fixedNode)
            {
                continue;
            }
            // 2. Actualización de su velocidad
            n.vel += h * (n.force / n.mass); // Según la 2ª Ley de Newton , F = m * a -> a = F / m 
            // 3. Actualización de su posición con la velocidad del paso actual
            n.pos += h * n.vel;
        }
    }

    private string[] LeerFichero(TextAsset fichero)
    {
        // Se pasa del fichero de texto a un array, que cuenta con el número de nodos y las posiciones de estos
        string[] texto = fichero.text.Split(new string[] { " ", "\n", "\r" }, System.StringSplitOptions.RemoveEmptyEntries);
        return texto;
    }

    private void AgregarNodos(TextAsset fichero, CultureInfo locale)
    {
        string[] textoNodos = LeerFichero(fichero);
        // Se accede a las posiciones de los vértices mediante el array obtenido
        int numNodos = int.Parse(textoNodos[0]); // Número de nodos (vértices de la malla)
        int numCoordenadas = int.Parse(textoNodos[1]); // Número de coordenadas por vértice
        int numCoordenadasNodos = numNodos * (numCoordenadas + 1); // Total de elementos que se deben recorrer
        // Se recorren las posiciones de los nodos -> SE INVIERTEN LOS EJES YZ
        for(int i = 4; i <= numCoordenadasNodos; i+= (numCoordenadas + 1))
        {
            int id = int.Parse(textoNodos[i]) - 1; // Se toma su identificador
            Vector3 posicionNodo = new Vector3(float.Parse(textoNodos[i + 1], locale), float.Parse(textoNodos[i + 3], locale), float.Parse(textoNodos[i + 2], locale)); // Se crea la posición del nodo, inviertiendo YZ
            Node nodo = new Node(id, transform.TransformPoint(posicionNodo), dAbs); // Creación del nodo, en coordenadas globales. Al identificador se le resta uno porque tetgen comienza en 1
            listaNodos.Add(nodo);
        }
    }

    private void CrearTetraedros(TextAsset fichero)
    {
        string[] textoTetraedros = LeerFichero(fichero);
        // Se acceden a los índices de los nodos de los tetraedros mediante el array obtenido
        int numTetraedros = int.Parse(textoTetraedros[0]); // Total de tetraedros que tiene la malla del envolvente
        int numVertices = int.Parse(textoTetraedros[1]); // Número de vértices de cada tetraedro (4)
        int numNodosTetraedros = numTetraedros * (numVertices + 1); // Total de elementos que se deben recorrer
        // Se recorren los índices de los nodos que componen los vértices de cada tetraedro
        for(int i= 3; i<= numNodosTetraedros; i+= (numVertices + 1))
        {
            int id = int.Parse(textoTetraedros[i]) - 1;
            // Obtención de los nodos de los vértices, previamente almacenados en la lista
            Node nodo1 = listaNodos[int.Parse(textoTetraedros[i + 1]) - 1]; // Se le resta 1 al índice del nodo obtenido, ya que se empieza a partir de 1 en lugar de 0 como en la lista
            Node nodo2 = listaNodos[int.Parse(textoTetraedros[i + 2]) - 1];
            Node nodo3 = listaNodos[int.Parse(textoTetraedros[i + 3]) - 1];
            Node nodo4 = listaNodos[int.Parse(textoTetraedros[i + 4]) - 1];
            // Creación del tetraedro
            Tetrahedron tetraedro = new Tetrahedron(id, nodo1, nodo2, nodo3, nodo4, p);
            // Se almacena en la lista de tetraedros
            listaTetraedros.Add(tetraedro);
        }
    }

    private void GenerarAristas()
    {
        // Para crear las aristas, se recorre la lista de tetraedros y se emparejan sus vértices formando 6 combinaciones en total (los tres lados de la base más los que unen con el vértice superior)
        for(int i = 0; i < listaTetraedros.Count; i++)
        {
            float volumenArista = listaTetraedros[i].volume / 6f;
            // Se ordenan sus vértices de menor a mayor, para facilitar la eliminación de aristas duplicadas
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

            // Se añaden las aristas a la lista
            aristas.Add(arista1);
            aristas.Add(arista2);
            aristas.Add(arista3);
            aristas.Add(arista4);
            aristas.Add(arista5);
            aristas.Add(arista6);
        }
        // Una vez creadas y añadidas a la lista, esta se ordena para detectar aristas duplicadas
        aristas.Sort();
    }

    private void CrearMuelles()
    {
        // Para crear los muelles, se debe recorrer la lista de aristas, evitando que haya más de un muelle para la misma arista. Sin embargo, debe almacenar el volumen acumulado
        // de las aristas duplicadas
        // Variable auxiliar para ir almacenando el volumen acumulado
        float volumenAcumulado = 0;
        for(int i = 0; i < aristas.Count - 1; i++)
        {
            // Si los extremos de la arista coinciden con la arista siguiente, no se crea un muelle, sólo se acumula el volumen
            if ((aristas[i].vA == aristas[i+1].vA) && (aristas[i].vB == aristas[i+1].vB))
            {
                volumenAcumulado += aristas[i].volumen;
            }
            else
            {
                volumenAcumulado += aristas[i].volumen;
                Spring muelle = new Spring(k, listaNodos[aristas[i].vA], listaNodos[aristas[i].vB], dDef, volumenAcumulado);
                volumenAcumulado = 0; // Se reinicia el contador del volumen acumulado
                listaMuelles.Add(muelle); // Se añade el muelle a la lista de muelles
            }
        }
        // Para evitar rebasar los límites de la lista, el último muelle se crea de forma manual
        Spring muelleFinal = new Spring(k, listaNodos[aristas[aristas.Count - 1].vA], listaNodos[aristas[aristas.Count - 1].vB], dDef, aristas[aristas.Count - 1].volumen);
        listaMuelles.Add(muelleFinal);
    }

    private void EstablecerVertices()
    {
        // Se obtienen los vértices de la malla (posiciones)
        Vector3[] posicionesVertices = malla.vertices;
        // Primero se identifica el tetraedro contenedor de cada vértice
        for(int i = 0; i < posicionesVertices.Length; i++)
        {
            // Para ello, se recorre la lista de tetraedros
            for(int j = 0; j < listaTetraedros.Count; j++)
            {
                if (listaTetraedros[j].Contains(posicionesVertices[i]))
                {
                    // Primero se crea el vértice junto con su tetraedro asociado
                    Vertex vertice = new Vertex(posicionesVertices[i], listaTetraedros[j]);
                    // Una vez hecho esto, se establecen sus coordenadas baricéntricas
                    vertice.CalcularCoordenadasBaricentricas();
                    listaVertices.Add(vertice);
                    break;
                }
            }
        }
    }

    // Función para calcular la nueva fuerza elástica según la densidad de los muelles
    private void applyElasticForce(Spring muelle)
    {
        // Ambos nodos de cada muelle sufren la misma fuerza, con sentidos opuestos
        // F = (-V / L0^2) * k * (L - L0) * (rA - rB) / L0
        muelle.nodeA.force += (-muelle.volume / (muelle.l0 * muelle.l0)) * muelle.k * (muelle.l - muelle.l0) * ((muelle.nodeA.pos - muelle.nodeB.pos) / muelle.l0);
        muelle.nodeB.force += -(-muelle.volume / (muelle.l0 * muelle.l0)) * muelle.k * (muelle.l - muelle.l0) * ((muelle.nodeA.pos - muelle.nodeB.pos) / muelle.l0);
    }

    // Función para aplicar el amortiguamiento en los muelles
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
