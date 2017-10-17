using System.Collections;
using System.Collections.Generic;
using System.Text;
using UnityEngine;

public class PoseUpdater : MonoBehaviour {

    [System.Serializable]
    struct Data
    {
        public float[] Position;
        public float[] Orientation;
    }

    public string UpdateTopic;

    // Use this for initialization
    void Start ()
    {
        MQTT.MQTTManager.Instance.Subscribe(UpdateTopic, OnUpdate);
    }

    void OnUpdate(string Topic, byte[] Data)
    {
        string DataAsString = Encoding.UTF8.GetString(Data);
        Data Update = JsonUtility.FromJson<Data>(DataAsString);

        Vector3 Position = new Vector3(-Update.Position[1], Update.Position[2], Update.Position[0]);
        Quaternion Orientation = new Quaternion(Update.Orientation[1], -Update.Orientation[2], -Update.Orientation[0], Update.Orientation[3]);

        transform.localPosition = Position;
        transform.localRotation = Orientation;

    }

}
