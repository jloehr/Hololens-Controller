using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SwapReceiver : MonoBehaviour {

    public string SwapTopic;
    public GameObject Controller;
    public GameObject Lightsaber;

    void Start()
    {
        MQTT.MQTTManager.Instance.Subscribe(SwapTopic, OnSwap);
    }

    void OnSwap(string Topic, byte[] Payload)
    {
        Debug.Log("Swap");

        bool ControllerActive = Controller.activeSelf;

        Controller.SetActive(!ControllerActive);
        Lightsaber.SetActive(ControllerActive);
    }
}
