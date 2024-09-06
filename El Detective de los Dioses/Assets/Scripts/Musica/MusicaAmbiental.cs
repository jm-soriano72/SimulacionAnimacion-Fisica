using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MusicaAmbiental : MonoBehaviour
{
    public AudioClip clipAudio;
    public AudioSource reproductorSonido;

    private void Awake()
    {
        reproductorSonido= GetComponent<AudioSource>();
    }

    // Start is called before the first frame update
    void Start()
    {
        reproductorSonido.clip = clipAudio;
        reproductorSonido.loop = true;
        reproductorSonido.Play();
    }

}
