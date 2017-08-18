package de.julianloehr.tangocontroller;


import com.google.atap.tango.mesh.TangoMesh;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

public class MeshUpdater implements MeshConstructor.OnTangoMeshesAvailableListener {
    private static final String UpdateTopic ="TangoController/RoomScan/Update";

    @Override
    public void onTangoMeshesAvailable(final TangoMesh[] meshes) {
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
            // Whatever
        }

        return data;
    }

    private JSONObject serializeMesh(final TangoMesh mesh)
    {
        JSONObject data = new JSONObject();

        try {
            JSONArray index = new JSONArray(mesh.index);
            JSONArray vertices = new JSONArray(mesh.vertices);
            JSONArray normales = new JSONArray(mesh.normals);

            data.put("Index", index);
            data.put("Verticex", vertices);
            data.put("Normals", normales);
        }
        catch (JSONException e)
        {
            // Whatever
        }

        return data;
    }
}
