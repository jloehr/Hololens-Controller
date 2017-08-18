package de.julianloehr.tangocontroller;

import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;

import com.google.atap.tangoservice.TangoCameraIntrinsics;
import com.google.atap.tangoservice.TangoPointCloudData;

public class ScanAlignment extends AppCompatActivity {
    public static final String TAG = ScanAlignment.class.getSimpleName();

    TangoWrapper tangoWrapper;
    MeshConstructor meshConstructor;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_scan_alignment);

        tangoWrapper = new TangoWrapper(this, new ShowsToastAndFinishOnUiThread(this));
        tangoWrapper.setOnTangoPoseAvailableListener( new PoseUpdater());

        meshConstructor = new MeshConstructor(new MeshUpdater());
        meshConstructor.setDepthCameraCalibration(tangoWrapper.getDepthCameraIntrinsics());
        tangoWrapper.setOnTangoPointCloudAvailableListener( meshConstructor );
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
