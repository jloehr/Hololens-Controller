package de.julianloehr.tangocontroller;

import android.util.Log;

import com.google.atap.tango.mesh.TangoMesh;
import com.google.atap.tangoservice.Tango;
import com.google.atap.tangoservice.TangoInvalidException;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.tango.support.TangoSupport;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.nio.FloatBuffer;

public class MeshUpdater implements TangoWrapper.OnTangoReadyListener, MeshConstructor.OnTangoMeshesAvailableListener {
    public static final String TAG = MeshUpdater.class.getSimpleName();
    private static final String UpdateTopic = "TangoController/RoomScan/Update";
    private static final String ResetTopic = "TangoController/RoomScan/Reset";

    public boolean Enabled = true;

    @Override
    public void onTangoReady(Tango tango) {
        Reset();
    }

    public void Reset()
    {
        TangoSupport.MatrixTransformData currentTransform = null;
        do {
            try {
                currentTransform = TangoSupport.getMatrixTransformAtTime(0,
                        TangoPoseData.COORDINATE_FRAME_AREA_DESCRIPTION,
                        TangoPoseData.COORDINATE_FRAME_DEVICE,
                        TangoSupport.ENGINE_TANGO,
                        TangoSupport.ENGINE_TANGO,
                        TangoSupport.ROTATION_IGNORED);
            } catch (TangoInvalidException e)
            {
                e.printStackTrace();
            }

            if(currentTransform == null)
            {
                Log.e(TAG, "No Pose!");
            }
        } while((currentTransform == null));

        JSONObject data = serializeTransform(currentTransform);

        MainActivity.mqttWrapper.Publish(ResetTopic, data.toString().getBytes(), 2);
    }

    @Override
    public void onTangoMeshesAvailable(final TangoMesh[] meshes) {
        if(meshes.length == 0)
            return;

        if(!Enabled)
            return;

        Log.i(TAG, "Updating Meshes: " + meshes.length );
        JSONObject data = serializeMeshes(meshes);
        MainActivity.mqttWrapper.Publish(UpdateTopic, data.toString().getBytes(), 1);

    }

    private JSONObject serializeTransform(final TangoSupport.MatrixTransformData matrixData)
    {
        JSONObject data = new JSONObject();
        try {
            data.put("DevicePoseTransform",  new JSONArray(matrixData.matrix));
        } catch (JSONException e)
        {
            Log.e(TAG, e.getMessage());
        }

        return data;
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
