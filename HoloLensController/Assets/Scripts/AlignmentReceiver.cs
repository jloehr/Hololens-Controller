using System.Collections;
using System.Collections.Generic;
using System.Text;
using UnityEngine;

public class AlignmentReceiver : MonoBehaviour
{
    public string AlignmentTopic;

    void Start () {
        MQTT.MQTTManager.Instance.Subscribe(AlignmentTopic, OnAlignment);
        gameObject.SetActive(false);
    }
	
    void OnAlignment(string Topic, byte[] Payload)
    {
        gameObject.SetActive(true);

        string AsString = Encoding.UTF8.GetString(Payload);
        Matrix4x4 Transformation = JsonUtility.FromJson<Matrix4x4>(AsString);
        
        Transformation.m02 *= -1f;
        Transformation.m12 *= -1f;
        Transformation.m20 *= -1f;
        Transformation.m21 *= -1f;
   
        Transformation.m23 *= -1f;
        
        transform.localPosition = Transformation.GetColumn(3);
        transform.localRotation = Quaternion.LookRotation(Transformation.GetColumn(2), Transformation.GetColumn(1));
    }
}
