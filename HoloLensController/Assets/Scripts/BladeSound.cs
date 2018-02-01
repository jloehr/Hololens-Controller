using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BladeSound : MonoBehaviour {

    public AudioSource OnSound;
    public AudioSource OffSound;
    public AudioSource HummingSound;

    public void On()
    {
        OnSound.Play();
        HummingSound.Play();
    }

    public void Off()
    {
        OffSound.Play();
        HummingSound.Stop();
    }
}
