using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

public class Lightsaber : MonoBehaviour {

    public AudioClip OnClip;
    public float OnClipFactor = 1f;
    public AudioClip OffClip;
    public float OffClipFactor = 1f;

    public UnityEvent OnOn;
    public UnityEvent OnOff;

    private Animator Animator;

    private bool OnState = false;

    
    void Awake () {
        Animator = GetComponent<Animator>();
    }
	
    public void Toggle()
    {
        Debug.Log("Toggle");
        if (!gameObject.activeInHierarchy)
            return;

        InternalToggle();
    }

    private void InternalToggle()
    {
        OnState = !OnState;
        Animator.SetBool("On", OnState);

        if (OnState)
            OnOn.Invoke();
        else
            OnOff.Invoke();
    }

    void OnEnable()
    {
        Debug.Log("OnEnable");
        Animator.SetFloat("OnSpeed", 1f / (OnClip.length * OnClipFactor));
        Animator.SetFloat("OffSpeed", 1f / (OffClip.length * OffClipFactor));
    }

    void OnDisable()
    {
        if (OnState)
            InternalToggle();
    }
}
