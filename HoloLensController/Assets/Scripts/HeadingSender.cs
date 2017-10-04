using MQTT;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using UnityEngine;

public class HeadingSender : MonoBehaviour {
    public string TangoResetTopic;
    public string HoloLensHeadingTopic;

    [Serializable]
    private struct Message {
        public float Angle;
        public float X;
        public float Y;
        public float Z;
    }

    void Start () {
        MQTTManager.Instance.Subscribe(TangoResetTopic, OnTangoReset);
	}

    void OnTangoReset(string Topic, byte[] Data)
    {
        Message Heading;
        Heading.Angle = transform.rotation.eulerAngles.y;
        Heading.X = transform.position.x;
        Heading.Y = transform.position.y;
        Heading.Z = transform.position.z;

        string AsString = JsonUtility.ToJson(Heading);
        byte[] AsByte = Encoding.UTF8.GetBytes(AsString);
        
        MQTTManager.Instance.Publish(HoloLensHeadingTopic, AsByte, false, MQTTManager.PublishType.ExactlyOnce);
    }
}
