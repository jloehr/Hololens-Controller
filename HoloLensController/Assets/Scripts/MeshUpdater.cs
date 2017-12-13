using MQTT;
using HoloToolkit.Unity.SpatialMapping;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Text;
using System;

public class MeshUpdater : MonoBehaviour {

    public string UpdateTopic;
    public string ClearTopic;

    private Dictionary<int, SpatialMappingSource.SurfaceObject> Work = new Dictionary<int, SpatialMappingSource.SurfaceObject>();

    // Use this for initialization
    void Start () {
        SpatialMappingObserver Observer = GetComponent<SpatialMappingObserver>();
        Observer.SurfaceAdded += OnSurfaceAdded;
        Observer.SurfaceUpdated += OnSurfaceUpdated;
        Observer.SurfaceRemoved += OnSurfaceRemoved;

        MQTT.MQTTManager.Instance.Publish(ClearTopic, null, false, MQTTManager.PublishType.ExactlyOnce);
    }

    void OnSurfaceAdded(object sender, HoloToolkit.Unity.DataEventArgs<SpatialMappingSource.SurfaceObject> e)
    {
        Debug.Log("Surface Added: " + e.Data.ID);
        Work[e.Data.ID] = e.Data;
    }

    void OnSurfaceUpdated(object sender, HoloToolkit.Unity.DataEventArgs<SpatialMappingSource.SurfaceUpdate> e)
    {
        Debug.Log("Surface Updated: " + e.Data.Old.ID + "/" +  e.Data.New.ID);
        Work[e.Data.New.ID] = e.Data.New;
    }

    void OnSurfaceRemoved(object sender, HoloToolkit.Unity.DataEventArgs<SpatialMappingSource.SurfaceObject> e)
    {
        Debug.Log("Surface Removed: " + e.Data.ID);
        Work[e.Data.ID] = e.Data;
    }

    void Update()
    {
        if(Work.Count > 0)
        {
            SendMeshes();
            Work.Clear();
        }
    }

    [Serializable]
    private struct Message
    {
        [Serializable]
        public struct Mesh
        {
            public int[] Index;
            public float[] Vertices;
            public float[] Normals;
        }

        public Mesh[] Meshes;
    }

    void SendMeshes()
    {
        Message MeshUpdate = new Message();
        MeshUpdate.Meshes = new Message.Mesh[Work.Count];
        int Index = 0;

        foreach(SpatialMappingSource.SurfaceObject Mesh in Work.Values)
        {
            MeshUpdate.Meshes[Index++] = PackMesh(Mesh);
        }

#if UNITY_WSA && !UNITY_EDITOR
        Windows.System.Threading.ThreadPool.RunAsync((workItem) => {
#endif

        string AsString = JsonUtility.ToJson(MeshUpdate);
        byte[] AsBytes = Encoding.UTF8.GetBytes(AsString);
        
        MQTT.MQTTManager.Instance.Publish(UpdateTopic, AsBytes, false, MQTTManager.PublishType.ExactlyOnce);

#if UNITY_WSA && !UNITY_EDITOR
        });
#endif
    }

    Message.Mesh PackMesh(SpatialMappingSource.SurfaceObject Mesh)
    {
        Message.Mesh NewMesh = new Message.Mesh();
        NewMesh.Index = new int[3];
        NewMesh.Index[0] = Mesh.ID;

        if(Mesh.Filter)
        {
            NewMesh.Vertices = PackFloats(Mesh.Filter.mesh.vertices, Mesh.Object.transform.localToWorldMatrix);
            Mesh.Filter.mesh.RecalculateNormals();
            NewMesh.Normals = PackFloats(Mesh.Filter.mesh.normals, Matrix4x4.TRS(Vector3.zero, Mesh.Object.transform.rotation, Vector3.one));
        }
       
        return NewMesh;
    }

    float[] PackFloats(Vector3[] Data, Matrix4x4 localToWorld)
    {
        float[] Array = new float[Data.Length * 3];
        int Index = 0;
        foreach (Vector3 Vector in Data)
        {
            Vector3 WorldSpace = localToWorld.MultiplyPoint3x4(Vector);

            Array[Index + 0] = WorldSpace.x;
            Array[Index + 1] = WorldSpace.y;
            Array[Index + 2] = WorldSpace.z;

            Index += 3;
        }

        return Array;
    }
}
