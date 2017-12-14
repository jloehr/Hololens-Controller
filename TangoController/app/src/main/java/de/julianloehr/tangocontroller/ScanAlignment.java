package de.julianloehr.tangocontroller;

import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;

import com.google.atap.tangoservice.TangoCameraIntrinsics;
import com.google.atap.tangoservice.TangoPointCloudData;

public class ScanAlignment extends AppCompatActivity {
    public static final String TAG = ScanAlignment.class.getSimpleName();
    private static final String DoneTopic = "TangoController/Controller/Done";
    private static final String SwapTopic = "TangoController/Controller/Swap";
    private static final String TapTopic = "TangoController/Controller/Tap";

    TangoWrapper tangoWrapper;
    MeshConstructor meshConstructor;
    MeshUpdater meshUpdater;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_scan_alignment);

        tangoWrapper = new TangoWrapper(this, new ShowsToastAndFinishOnUiThread(this));
        tangoWrapper.setOnTangoPoseAvailableListener( new PoseUpdater());

        meshUpdater = new MeshUpdater();
        meshConstructor = new MeshConstructor(meshUpdater);

        tangoWrapper.setOnTangoPointCloudAvailableListener( meshConstructor );
        tangoWrapper.addOnTangoreadyListener( meshConstructor );
        tangoWrapper.addOnTangoreadyListener( meshUpdater );
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

    public void onDoneButtonClick(View v)
    {
        meshUpdater.Enabled = false;
        MainActivity.mqttWrapper.Publish(DoneTopic, null, 2);

        Button DoneButton = (Button) findViewById(R.id.done);
        DoneButton.setVisibility(View.GONE);
        Button SwapButton = (Button) findViewById(R.id.swap);
        SwapButton.setVisibility(View.VISIBLE);
    }

    public void onSwapButtonClick(View v)
    {
        MainActivity.mqttWrapper.Publish(SwapTopic, null, 2);
    }

    public void onTapButtonClick(View v)
    {
        MainActivity.mqttWrapper.Publish(TapTopic, null, 2);
    }
}
