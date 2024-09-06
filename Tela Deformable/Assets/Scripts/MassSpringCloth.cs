using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MassSpringCloth : MonoBehaviour
{
    Mesh bandera; // Referencia al mesh de la bandera
    // Variables que almacenan la información del plano (posición de sus vértices y sus triángulos
    public Vector3[] vertices;
    public int[] triangulos;

    // Lista que va a guardar los nodos de la tela, que son vértices con una masa determinada
    public List<Node> nodos;
    // Lista que almacena todos los muelles que conectan los nodos de la tela
    public List<Spring> muelles;

    // Para poder identificar todos los muelles de la tela, se utiliza una estructura de datos
    [System.Serializable]
    public struct Edge : IComparable<Edge>
    {
        public int vertexA;
        public int vertexB;
        public int vertexOther;

        public Edge(int a, int b, int c)
        {
            vertexA = a;
            vertexB = b;
            vertexOther = c;
        }
        // Se implementa la interfaz comparable, para poder ordenar una lista de aristas en el futuro, y así identificar aristas duplicadas
        public int CompareTo(Edge other)
        {
            // Primer se compara el vértice A
            if (vertexA != other.vertexA)
                return vertexA.CompareTo(other.vertexA);
            // Si son iguales, se compara el vértice B
            return vertexB.CompareTo(other.vertexB);
        }
    }

    public List<Edge> aristas = new List<Edge>(); // Lista que va a almacenar todas las aristas

    // PARÁMETROS

    // Se crean los tipos de los muelles, con sus constantes de rigidez asociadas

    public float k_Traccion = 10f;
    public float k_Flexion = 1f;

    public float masaTotal = 10f;
    public Vector3 g = new Vector3(0, -9.8f, 0); // Vector del campo gravitatorio terrestre

    // Factores de amortiguamiento
    public float dAbsolute = 0.3f;
    public float dDeformation = 1f;

    public enum Integration
    {
        ExplicitEuler = 0,
        SimplecticEuler = 1
    }

    public Integration integrationMethod; // Variable que en el inspector selecciona el método de integración numérica a realizar
    public float h = 0.001f; // Paso del tiempo

    bool paused;

    // VECTOR PARA INDICAR LA FUERZA DEL VIENTO

    public Vector3 direccionViento = new Vector3 (0, 0, 1); // Vector director
    public float intensidadViento = 0.3f;
    private float tiempoTranscurrido = 0;

    // Start is called before the first frame update
    void Start()
    {
        direccionViento = direccionViento.normalized; // Se normaliza la dirección del viento
        paused = true; // En un primer momento, la animación está pausada

        // En este método se inicializan las variables de los vértices, triángulos y nodos según los valores iniciales del plano
        bandera = this.GetComponent<MeshFilter>().mesh;

        vertices = bandera.vertices;
        triangulos = bandera.triangles;

        // Una vez obtenidos los datos de los vértices, se crean los nodos
        nodos = new List<Node>(vertices.Length);
        foreach(Vector3 v in vertices)
        {
            nodos.Add(new Node((masaTotal/vertices.Length), transform.TransformPoint(v), dAbsolute)); // La masa de cada nodo será la misma -> masaTotal / numNodos
            // Para obtener la coordenada global del nodo, se utiliza TransformPoint. De esta forma, se comprobará si es fijo o no
        }

        // Después de haber creado los nodos, se procede a crear los muelles. Para ello, es necesario identificar las aristas
        inicializarMuelles();

    }

    // Update is called once per frame
    void Update()
    {
        // Se controla si se quiere pasar o reanudar la animación, empleando la tecla P
        if(Input.GetKeyDown(KeyCode.P))
        {
            paused = !paused;
        }

        bandera.MarkDynamic();
        Vector3[] newVertex = bandera.vertices;

        // La transformación se aplica al vértice de la malla, en coordenadas locales
        for(int i=0; i< newVertex.Length; i++)
        {
            newVertex[i] = transform.InverseTransformPoint(nodos[i].pos);
        }
        bandera.MarkDynamic();
        bandera.vertices = newVertex;
    }

    private void FixedUpdate()
    {
        if(paused)
        {
            return;
        }
        // En función del método de integración escogido, se utiliza una función u otra
        switch(integrationMethod)
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
        foreach (Spring muelle in muelles)
        {
            // Dirección
            muelle.dir = (muelle.nodeA.pos - muelle.nodeB.pos).normalized;
            // Nueva longitud
            muelle.l = (muelle.nodeA.pos - muelle.nodeB.pos).magnitude;
        }

        tiempoTranscurrido += Time.fixedDeltaTime;
    }

    private void integrationExplicitEuler()
    {
        // Primero se recorre la lista de nodos para aplicarles las fuerzas individuales que sufren (peso y amortiguamiento)
        foreach(Node n in nodos)
        {
            // Reinicio de la fuerza
            n.force = Vector3.zero;

            // Si el nodo es fijo, no se actualiza su posición
            if(n.fixedNode)
            {
                continue;
            }
            // 1 - Actualización de su posición con la velocidad del paso anterior
            n.pos += h * n.vel;

            // 2 - Cálculo de fuerzas
            n.force += (g * n.mass) - (n.dAbsolute * n.vel * n.mass); // Peso (P = m * g) y amortiguamiento (F = - d * m * v)
            // FUERZA DEL VIENTO
            n.force += fuerzaViento(tiempoTranscurrido);
        }
        // Después, se recorre la lista de muelles, para aplicarles a todos los nodos las fuerzas elásticas y de amortiguamiento debido a estos
        foreach(Spring muelle in muelles)
        {
            // Fuerza elástica -> F = - k * (L-L0) * d -> Tiene sentidos opuestos para los dos extremos del muele
            muelle.nodeA.force += -muelle.k * (muelle.l - muelle.l0) * muelle.dir;
            muelle.nodeB.force += muelle.k * (muelle.l - muelle.l0) * muelle.dir;

            // Fuerza de amortiguamiento debida a a la velocidad relativa entre los dos nodos del muelle
            // applyDampingSpring(muelle);
        }
        // 3 - Una vez calculadas todas las fuerzas de cada nodo, se actualiza su velocidad
        foreach(Node n in nodos)
        {
            if(n.fixedNode)
            {
                continue;
            }
            n.vel += h * (n.force / n.mass); // Según la 2ª Ley de Newton , F = m * a -> a = F / m 
        }
    }

    private void integrationSimplecticEuler()
    {
        // Primero se recorre la lista de nodos para aplicarles las fuerzas individuales que sufren (peso y amortiguamiento)
        foreach (Node n in nodos)
        {
            // 1 - Cálculo de fuerzas 
            // Reinicio de la fuerza
            n.force = Vector3.zero;
            // Cálculo de la fuerza sobre un nodo
            n.force += (g * n.mass) - (n.dAbsolute * n.vel * n.mass); // Peso (P = m * g) y amortiguamiento (F = - d * m * v)
            // FUERZA DEL VIENTO
            n.force += fuerzaViento(tiempoTranscurrido);
        }
        // Después, se recorre la lista de muelles, para aplicarles a todos los nodos las fuerzas elásticas y de amortiguamiento debido a estos
        
        foreach (Spring muelle in muelles)
        {
            // Fuerza elástica -> F = - k * (L-L0) * d -> Tiene sentidos opuestos para los dos extremos del muele
            muelle.nodeA.force += -muelle.k * (muelle.l - muelle.l0) * muelle.dir;
            muelle.nodeB.force += muelle.k * (muelle.l - muelle.l0) * muelle.dir;

            // Fuerza de amortiguamiento debida a a la velocidad relativa entre los dos nodos del muelle
            applyDampingSpring(muelle);
        }
        

        foreach(Node n in nodos)
        {
            // Si el nodo es fijo, no se actualizan su posición y velocidad
            if(n.fixedNode)
            {
                continue;
            }
            // 2. Actualización de su velocidad
            n.vel += h * (n.force / n.mass); // Según la 2ª Ley de Newton , F = m * a -> a = F / m 
            // 3. Actualización de su posición con la velocidad del paso actual
            n.pos += h * n.vel;
        }
    }

    private void inicializarMuelles()
    {
        // Para ir creando las aristas correspondientes a cada triángulo, se recorre el array de triángulos del mesh de 3 en 3, ya que, almacena los índices de sus triángulos de 3 en 3
        for(int i=0; i<triangulos.Length; i+=3)
        {
            // Primero se obtienen los índices de los vértices de cada triángulo
            int vertexA = triangulos[i];
            int vertexB = triangulos[i+1];
            int vertexC = triangulos[i+2];

            // Después, se crean las aristas del triángulo con dichos vértices -> SE DEBE CONTROLAR QUE VERTEX A TIENE QUE SER MENOR A VERTEX B, para facilitar la búsqueda de aristas repetidas
            Edge arista_AB, arista_BC, arista_AC;
            if(vertexA < vertexB)
            {
                arista_AB = new Edge(vertexA, vertexB, vertexC);
            }
            else
            {
                arista_AB = new Edge(vertexB, vertexA, vertexC);
            }

            if(vertexB < vertexC)
            {
                arista_BC = new Edge(vertexB, vertexC, vertexA);
            }
            else
            {
                arista_BC = new Edge(vertexC, vertexB, vertexA);
            }

            if(vertexA < vertexC)
            {
                arista_AC = new Edge(vertexA, vertexC, vertexB);
            }
            else
            {
                arista_AC = new Edge(vertexC, vertexA, vertexA);
            }

            // Por último, se añaden las aristas a la lista
            aristas.Add(arista_AB);
            aristas.Add(arista_BC);
            aristas.Add(arista_AC);
        }
        // Una vez añadidas todas las aristas, se ordena la lista para poder detectar aristas duplicadas y crear los muelles
        aristas.Sort();

        for(int i=0; i<aristas.Count-1; i++)
        {
            // Si una arista es igual que la siguiente, quiere decir que está duplicada, por lo que se debe crear un MUELLE DE FLEXIÓN
            if ((aristas[i].vertexA == aristas[i+1].vertexA) && (aristas[i].vertexB == aristas[i+1].vertexB))
            {
                Spring flexion = new Spring(k_Flexion, nodos[aristas[i].vertexOther], nodos[aristas[i + 1].vertexOther], dDeformation); // Se crea un muelle de flexión entre los vértices
                // opuestos de la arista compartida
                muelles.Add(flexion);
            }
            // Si no, se debe crear un MUELLE DE TRACCIÓN
            else
            {
                Spring traccion = new Spring(k_Traccion, nodos[aristas[i].vertexA], nodos[aristas[i].vertexB], dDeformation); // Se crea un muelle de tracción entre los vértices de la arista
                muelles.Add(traccion);
            }
        }
        // Como no se puede acceder al elemento siguiente de la última arista, se crea el último muelle de tracción una vez se haya terminado
        muelles.Add(new Spring(k_Traccion, nodos[aristas[aristas.Count-1].vertexA], nodos[aristas[aristas.Count-1].vertexB], dDeformation));
    }
    
    // Función para dibujar las líneas de los muelles
    private void OnDrawGizmos()
    {
        // Se recorre la lista de muelles para dibujarlos
        foreach(Spring muelle in muelles)
        {
            // Mediante su constante de rigidez, se comprueba si los muelles son de tracción o de flexión, para indicar el color de la línea
            if(muelle.k == k_Traccion)
            {
                Gizmos.color = Color.red;
            }
            else
            {
                Gizmos.color = Color.blue;
            }
            // Es importante utilizar las coordenadas globales de los nodos para dibujar las líneas correctamente
            Gizmos.DrawLine(muelle.nodeA.pos, muelle.nodeB.pos);
        }
    }

    // Función para aplicar el amortiguamiento en los muelles
    private void applyDampingSpring(Spring muelle)
    {
        // Ambos nodos de cada muelle sufren la misma fuerza, con sentidos opuestos
        // F = - d * [(vA - VB) * u)] * u
        muelle.nodeA.force += -muelle.dDeformation * Vector3.Dot((muelle.nodeA.vel - muelle.nodeB.vel), muelle.dir) * muelle.dir;
        muelle.nodeB.force += muelle.dDeformation * Vector3.Dot((muelle.nodeA.vel - muelle.nodeB.vel), muelle.dir) * muelle.dir;
    }

    // Función para simular el viento con una función sinusoidal (movimiento ondulatorio)
    private Vector3 fuerzaViento(float tiempo)
    {
        // Parámetros de la onda
        float amplitud = 0.05f; // Amplitud máxima de la onda
        float velocidadAngular = 0.35f; // Velocidad angular (rad/s)
        float faseInicial = 0f; // Fase inicial

        // Cálculo de la variación de la fuerza del viento con la ecuación de una onda
        float variacionFuerza = amplitud * (float) Math.Sin(velocidadAngular * tiempo + faseInicial);

        // Aplicación de la variación de la intensidad del viento
        float intensidadVientoModificada = intensidadViento + variacionFuerza;

        return intensidadVientoModificada * direccionViento;
    }
}
