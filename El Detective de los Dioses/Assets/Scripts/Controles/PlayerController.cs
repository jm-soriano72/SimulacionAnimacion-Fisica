using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlayerController : MonoBehaviour
{
    private Animator _animatorPersonaje;
    [SerializeField] private float velocidad = 0.75f;
    [SerializeField] private float velocidadCorrer = 1.5f;
    [SerializeField] private float velocidadRotacion = 70f;

    public GameObject pistola;

    private void Awake()
    {
        _animatorPersonaje = GetComponentInChildren<Animator>();
    }

    // Update is called once per frame
    void Update()
    {

        // Control de la visibilidad de la pistola
        // Obtener el estado actual del Animator Controller
        AnimatorStateInfo estadoActual = _animatorPersonaje.GetCurrentAnimatorStateInfo(0);

        // Verificar el nombre del estado actual
        if (estadoActual.IsName("Disparar"))
        {
            pistola.SetActive(true);
        }
        else
        {
            pistola.SetActive(false);
        }

        // Movimiento hacia adelante con la tecla W
        if (Input.GetKey(KeyCode.W))
        {
            float currentMoveSpeed = Input.GetKey(KeyCode.LeftShift) ? velocidadCorrer : velocidad;
            transform.Translate(Vector3.forward * currentMoveSpeed * Time.deltaTime);
            _animatorPersonaje.SetBool("isWalking", true);
            if(currentMoveSpeed == velocidadCorrer)
            {
                _animatorPersonaje.SetBool("isRunning", true);
            }
            else
            {
                _animatorPersonaje.SetBool("isRunning", false);
            }
        }
        else
        {
            _animatorPersonaje.SetBool("isWalking", false);
            _animatorPersonaje.SetBool("isRunning", false);
        }

        // Rotación con las teclas A y D
        if (Input.GetKey(KeyCode.A))
        {
            transform.Rotate(Vector3.up, - velocidadRotacion * Time.deltaTime);
        }
        else if (Input.GetKey(KeyCode.D))
        {
            transform.Rotate(Vector3.up, velocidadRotacion * Time.deltaTime);
        }

        // Disparar
        if (Input.GetKeyDown(KeyCode.Q))
        {
            _animatorPersonaje.SetTrigger("isShooting");
        }

        // Pegar patada
        if(Input.GetKeyDown(KeyCode.E))
        {
            _animatorPersonaje.SetTrigger("isKicking");
        }

        // Salto mortal
        if(Input.GetKeyDown(KeyCode.Space))
        {
            _animatorPersonaje.SetTrigger("isJumping");
        }
    }
}
