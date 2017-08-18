package de.julianloehr.tangocontroller;

import android.content.Intent;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Toast;

import com.google.atap.tangoservice.TangoPoseData;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;


public class MainActivity extends AppCompatActivity {

    public static final String TAG = MainActivity.class.getSimpleName();
    public static MQTTWrapper mqttWrapper;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        mqttWrapper = new MQTTWrapper(this);
        mqttWrapper.Start();
    }

    public void onVisualAlignmentButtonClick(View v)
    {
        switchActivity(VisualAlignment.class);
    }

    public void onScanAlignmentButtonClick(View v)
    {
        switchActivity(ScanAlignment.class);
    }

    private void switchActivity(Class activity)
    {
        Intent intent = new Intent(this, activity);
        startActivity(intent);
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
