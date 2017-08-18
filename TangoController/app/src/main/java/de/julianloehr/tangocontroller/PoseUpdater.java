package de.julianloehr.tangocontroller;

import android.util.Log;

import com.google.atap.tangoservice.TangoPoseData;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

public class PoseUpdater implements TangoWrapper.OnTangoPoseAvailableListener {
    public static final String TAG = VisualAlignment.class.getSimpleName();
    private static final String UpdateTopic ="TangoController/Controller/Update";

    @Override
    public void onTangoPoseAvailable(final TangoPoseData pose)
    {
        logPose(pose, TAG);
        JSONObject Data = serializePose(pose);
        MainActivity.mqttWrapper.Publish(UpdateTopic, Data.toString().getBytes());
    }

    private JSONObject serializePose(final TangoPoseData pose)
    {
        float translation[] = pose.getTranslationAsFloats();
        float orientation[] = pose.getRotationAsFloats();

        JSONObject Data = new JSONObject();
        JSONArray Position = new JSONArray();
        JSONArray Orientation = new JSONArray();

        try {
            Position.put((double)translation[0]);
            Position.put((double)translation[1]);
            Position.put((double)translation[2]);

            Orientation.put((double)orientation[0]);
            Orientation.put((double)orientation[1]);
            Orientation.put((double)orientation[2]);
            Orientation.put((double)orientation[3]);

            Data.put("Position", (Object) Position);
            Data.put("Orientation", (Object) Orientation);
        } catch (JSONException ex)
        {

        }

        return Data;
    }

    /**
     * Log the Position and Orientation of the given pose in the Logcat as information.
     *
     * @param pose the pose to log.
     */
    private void logPose(final TangoPoseData pose, String tag) {
        StringBuilder stringBuilder = new StringBuilder();

        float translation[] = pose.getTranslationAsFloats();
        stringBuilder.append("Position: " +
                translation[0] + ", " + translation[1] + ", " + translation[2]);

        float orientation[] = pose.getRotationAsFloats();
        stringBuilder.append(". Orientation: " +
                orientation[0] + ", " + orientation[1] + ", " +
                orientation[2] + ", " + orientation[3]);

        Log.i(tag, stringBuilder.toString());
    }
}
