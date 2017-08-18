package de.julianloehr.tangocontroller;

import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.widget.Toast;

import com.google.atap.tangoservice.TangoPoseData;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

public class VisualAlignment extends AppCompatActivity implements TangoWrapper.ShowsToastAndFinishOnUiThreadInterface, TangoWrapper.OnTangoPoseAvailableListener {
    public static final String TAG = VisualAlignment.class.getSimpleName();
    private static final String UpdateTopic ="TangoController/Controller/Update";

    TangoWrapper tangoWrapper; // = new TangoWrapper(this);

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_visual_alignment);

        tangoWrapper = new TangoWrapper(this, this , this);
    }

    @Override
    protected void onResume() {
        super.onResume();

        tangoWrapper.Start();
    }

    @Override
    protected void onStop()
    {
        super.onStop();

        tangoWrapper.Stop();
    }

    public void OnTangoPoseAvailable(final TangoPoseData pose)
    {
        tangoWrapper.logPose(pose, TAG);

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

        MainActivity.mqttWrapper.Publish(UpdateTopic, Data.toString().getBytes());
    }

    /**
     * Display toast on UI thread.
     *
     * @param resId The resource id of the string resource to use. Can be formatted text.
     */
    public void showsToastAndFinishOnUiThread(final int resId) {
        showsToastAndFinishOnUiThread(getString(resId));
    }

    public void showsToastAndFinishOnUiThread(final String text) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(VisualAlignment.this,
                        text, Toast.LENGTH_LONG).show();
                finish();
            }
        });
    }
}
