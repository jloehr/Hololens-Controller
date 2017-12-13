using System.Collections;
using System.Collections.Generic;
using System.Text;
using UnityEngine;
using UnityEngine.Profiling;

public class PoseUpdater : MonoBehaviour {

    [System.Serializable]
    struct Data
    {
        public float[] Position;
        public float[] Orientation;
    }

    public string UpdateTopic;
    private byte[] Buffer = null;

    // Use this for initialization
    void Start ()
    {
        MQTT.MQTTManager.Instance.Subscribe(UpdateTopic, OnUpdate);
    }

    void OnUpdate(string Topic, byte[] Data)
    {
        Buffer = Data;
    }

    void Update()
    {
        if(Buffer != null)
        {
            Profiler.BeginSample("Pose Updater");

            Debug.Log("Pose received on Frame " + Time.frameCount);

            string DataAsString = Encoding.UTF8.GetString(Buffer);
            Data Update = JsonUtility.FromJson<Data>(DataAsString);

            Vector3 Position = new Vector3(Update.Position[0], Update.Position[2], Update.Position[1]);
            Quaternion Orientation = new Quaternion(-Update.Orientation[0], -Update.Orientation[2], -Update.Orientation[1], Update.Orientation[3]);

            transform.localPosition = Position;
            transform.localRotation = Orientation;

            Buffer = null;
            Profiler.EndSample();
        }
    }

}
