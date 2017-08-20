package de.julianloehr.tangocontroller;


import android.util.Log;

import com.google.atap.tango.mesh.TangoMesh;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.nio.FloatBuffer;

public class MeshUpdater implements MeshConstructor.OnTangoMeshesAvailableListener {
    public static final String TAG = MeshUpdater.class.getSimpleName();
    private static final String UpdateTopic ="TangoController/RoomScan/Update";

    @Override
    public void onTangoMeshesAvailable(final TangoMesh[] meshes) {
        if(meshes.length == 0)
            return;

        Log.i(TAG, "Updating Meshes: " + meshes.length );
        JSONObject data = serializeMeshes(meshes);
        MainActivity.mqttWrapper.Publish(UpdateTopic, data.toString().getBytes());

    }

    private JSONObject serializeMeshes(final TangoMesh[] meshes)
    {
        JSONObject data = new JSONObject();
        JSONArray JSONMeshes = new JSONArray();

        for (TangoMesh mesh: meshes) {
            JSONMeshes.put(serializeMesh(mesh));
        }

        try {
            data.put("Meshes", JSONMeshes);
        }
        catch (JSONException e)
        {
            Log.e(TAG, e.getMessage());
        }

        return data;
    }

    private JSONObject serializeMesh(final TangoMesh mesh)
    {
        JSONObject data = new JSONObject();

        try {
            JSONArray index = new JSONArray(mesh.index);
            JSONArray vertices = serializeFloatBuffer(mesh.vertices, mesh.numVertices);
            JSONArray normales = serializeFloatBuffer(mesh.normals, mesh.numVertices);

            data.put("Index", index);
            data.put("Vertices", vertices);
            data.put("Normals", normales);
        }
        catch (JSONException e)
        {
            Log.e(TAG, e.getMessage());
        }

        return data;
    }

    private JSONArray serializeFloatBuffer(FloatBuffer buffer, int numTriples)
            throws JSONException
    {
        if(buffer.hasArray()) {
            return new JSONArray(buffer.array());
        }
        else
        {
            float[] tmpBufer = new float[numTriples * 3];
            buffer.get(tmpBufer);
            return new JSONArray(tmpBufer);
        }
    }
}
