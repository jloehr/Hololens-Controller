package de.julianloehr.tangocontroller;

import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.widget.Toast;

import com.google.atap.tangoservice.TangoPoseData;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;


public class MainActivity extends AppCompatActivity {

    public static final String TAG = MainActivity.class.getSimpleName();

    TangoWrapper tangoWrapper = new TangoWrapper(this);
    MQTTWrapper mqttWrapper = new MQTTWrapper(this);

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);


        mqttWrapper.Start();
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
        //mqttWrapper.Stop();
    }

    public void consumePose(TangoPoseData pose)
    {
        logPose(pose);

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

        mqttWrapper.SendUpdate(Data.toString().getBytes());
    }

    /**
     * Log the Position and Orientation of the given pose in the Logcat as information.
     *
     * @param pose the pose to log.
     */
    private void logPose(TangoPoseData pose) {
        StringBuilder stringBuilder = new StringBuilder();

        float translation[] = pose.getTranslationAsFloats();
        stringBuilder.append("Position: " +
                translation[0] + ", " + translation[1] + ", " + translation[2]);

        float orientation[] = pose.getRotationAsFloats();
        stringBuilder.append(". Orientation: " +
                orientation[0] + ", " + orientation[1] + ", " +
                orientation[2] + ", " + orientation[3]);

        Log.i(TAG, stringBuilder.toString());
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
                Toast.makeText(MainActivity.this,
                        text, Toast.LENGTH_LONG).show();
                finish();
            }
        });
    }
}
