package de.julianloehr.tangocontroller;

import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.widget.Toast;

import com.google.atap.tangoservice.TangoPoseData;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

public class VisualAlignment extends AppCompatActivity {

    TangoWrapper tangoWrapper;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_visual_alignment);

        tangoWrapper = new TangoWrapper(this, new ShowsToastAndFinishOnUiThread(this));
        tangoWrapper.setOnTangoPoseAvailableListener( new PoseUpdater());
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

}
