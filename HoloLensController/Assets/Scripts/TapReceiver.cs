using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TapReceiver : MonoBehaviour {

    public string TapTopic;

    void Start()
    {
        MQTT.MQTTManager.Instance.Subscribe(TapTopic, OnTap);

    }

    void OnTap(string Topic, byte[] Payload)
    {
        Debug.Log("Tap");
    }
}
