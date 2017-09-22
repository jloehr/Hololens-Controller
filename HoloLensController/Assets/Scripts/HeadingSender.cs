using MQTT;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using UnityEngine;

public class HeadingSender : MonoBehaviour {
    public string TangoResetTopic;
    public string HoloLensHeadingTopic;

    void Start () {
        MQTTManager.Instance.Subscribe(TangoResetTopic, OnTangoReset);
	}

    void OnTangoReset(string Topic, byte[] Data)
    {
        Matrix4x4 Heading = transform.worldToLocalMatrix;
        string AsString = JsonUtility.ToJson(Heading);
        byte[] AsByte = Encoding.UTF8.GetBytes(AsString);
        
        MQTTManager.Instance.Publish(HoloLensHeadingTopic, AsByte, false, MQTTManager.PublishType.ExactlyOnce);
    }
}
