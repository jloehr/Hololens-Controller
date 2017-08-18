package de.julianloehr.tangocontroller;

import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.widget.Toast;

import com.google.atap.tangoservice.TangoPoseData;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

public class VisualAlignment extends AppCompatActivity implements TangoWrapper.ShowsToastAndFinishOnUiThreadInterface {

    TangoWrapper tangoWrapper;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_visual_alignment);

        tangoWrapper = new TangoWrapper(this, this , new PoseUpdater());
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
