using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

public class TapReceiver : MonoBehaviour {

    public string TapTopic;
    public UnityEvent OnTap;

    void Start()
    {
        MQTT.MQTTManager.Instance.Subscribe(TapTopic, OnTapCallback);
    }

    void OnTapCallback(string Topic, byte[] Payload)
    {
        Debug.Log("Tap");
        OnTap.Invoke();
    }
}
