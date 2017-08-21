package de.julianloehr.tangocontroller;

import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;

import com.google.atap.tangoservice.TangoCameraIntrinsics;
import com.google.atap.tangoservice.TangoPointCloudData;

public class ScanAlignment extends AppCompatActivity {
    public static final String TAG = ScanAlignment.class.getSimpleName();

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
        tangoWrapper.setOnTangoreadyListener( meshConstructor );
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

    public void onClearButtonClick(View v)
    {
        meshUpdater.clear();
    }


}
