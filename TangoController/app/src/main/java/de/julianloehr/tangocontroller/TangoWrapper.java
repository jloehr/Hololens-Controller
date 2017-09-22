package de.julianloehr.tangocontroller;

import android.content.Context;
import android.util.Log;

import com.google.atap.tangoservice.Tango;
import com.google.atap.tangoservice.TangoCameraIntrinsics;
import com.google.atap.tangoservice.TangoConfig;
import com.google.atap.tangoservice.TangoCoordinateFramePair;
import com.google.atap.tangoservice.TangoErrorException;
import com.google.atap.tangoservice.TangoEvent;
import com.google.atap.tangoservice.TangoInvalidException;
import com.google.atap.tangoservice.TangoOutOfDateException;
import com.google.atap.tangoservice.TangoPointCloudData;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.atap.tangoservice.TangoXyzIjData;

import java.util.ArrayList;

public class TangoWrapper {

    public interface ShowsToastAndFinishOnUiThreadInterface {
        void showsToastAndFinishOnUiThread(final int resId);
        void showsToastAndFinishOnUiThread(final String text);
    }

    public interface OnTangoReadyListener {
        void onTangoReady(Tango tango);
    }

    public interface OnTangoPoseAvailableListener {
        void onTangoPoseAvailable(final TangoPoseData pose);
    }

    public interface OnTangoPointCloudAvailableListener {
        void onTangoPointCloudAvailable(final TangoPointCloudData pointCloud);
    }

    private Context mContext;
    private ShowsToastAndFinishOnUiThreadInterface mShowToastInterface;
    private OnTangoReadyListener mTangoReadyCallback;
    private OnTangoPoseAvailableListener mPoseCallback;
    private OnTangoPointCloudAvailableListener mPointCloudCallback;

    private com.google.atap.tangoservice.Tango mTango;
    private TangoConfig mConfig;


    public TangoWrapper(Context context, ShowsToastAndFinishOnUiThreadInterface showToastInterface)
    {
        mContext = context;
        mShowToastInterface = showToastInterface;
    }

    void setOnTangoreadyListener(OnTangoReadyListener tangoReadyCallback)
    {
        mTangoReadyCallback = tangoReadyCallback;
    }

    public void setOnTangoPoseAvailableListener(OnTangoPoseAvailableListener poseCallback)
    {
        mPoseCallback = poseCallback;
    }

    public void setOnTangoPointCloudAvailableListener(OnTangoPointCloudAvailableListener PointCloudCallback)
    {
        mPointCloudCallback = PointCloudCallback;
    }

    public void Start()
    {
        // Initialize Tango Service as a normal Android Service. Since we call mTango.disconnect()
        // in onPause, this will unbind Tango Service, so every time onResume gets called we
        // should create a new Tango object.
        mTango = new Tango(mContext, new Runnable() {
            // Pass in a Runnable to be called from UI thread when Tango is ready; this Runnable
            // will be running on a new thread.
            // When Tango is ready, we can call Tango functions safely here only when there are no
            // UI thread changes involved.
            @Override
            public void run() {
                synchronized (TangoWrapper.this) {
                    try {
                        mConfig = setupTangoConfig(mTango);
                        mTango.connect(mConfig);
                        startupTango();
                        if(mTangoReadyCallback != null)
                            mTangoReadyCallback.onTangoReady(mTango);

                        TangoSupport.initialize(mTango);
                    } catch (TangoOutOfDateException e) {
                        Log.e(MainActivity.TAG, mContext.getString(R.string.exception_out_of_date), e);
                        mShowToastInterface.showsToastAndFinishOnUiThread(R.string.exception_out_of_date);
                    } catch (TangoErrorException e) {
                        Log.e(MainActivity.TAG, mContext.getString(R.string.exception_tango_error), e);
                        mShowToastInterface.showsToastAndFinishOnUiThread(R.string.exception_tango_error);
                    } catch (TangoInvalidException e) {
                        Log.e(MainActivity.TAG, mContext.getString(R.string.exception_tango_invalid), e);
                        mShowToastInterface.showsToastAndFinishOnUiThread(R.string.exception_tango_invalid);
                    }
                }
            }
        });
    }

    public void Stop()
    {
        synchronized (TangoWrapper.this) {
            try {
                mTango.disconnect();
            } catch (TangoErrorException e) {
                Log.e(MainActivity.TAG, mContext.getString(R.string.exception_tango_error), e);
            }
        }
    }

    /**
     * Sets up the tango configuration object. Make sure mTango object is initialized before
     * making this call.
     */
    private TangoConfig setupTangoConfig(com.google.atap.tangoservice.Tango tango) {
        // Create a new Tango Configuration and enable the HelloMotionTrackingActivity API.
        TangoConfig config = tango.getConfig(TangoConfig.CONFIG_TYPE_DEFAULT);
        if(mPoseCallback != null)
        {
            config.putBoolean(TangoConfig.KEY_BOOLEAN_MOTIONTRACKING, true);
            config.putBoolean(TangoConfig.KEY_BOOLEAN_DRIFT_CORRECTION, false);
            config.putBoolean(TangoConfig.KEY_BOOLEAN_LEARNINGMODE, true);
        }

        if(mPointCloudCallback != null)
        {
            config.putBoolean(TangoConfig.KEY_BOOLEAN_LOWLATENCYIMUINTEGRATION, true);
            config.putBoolean(TangoConfig.KEY_BOOLEAN_SMOOTH_POSE, true);
            config.putBoolean(TangoConfig.KEY_BOOLEAN_DEPTH, true);
            config.putInt(TangoConfig.KEY_INT_DEPTH_MODE, TangoConfig.TANGO_DEPTH_MODE_POINT_CLOUD);
        }

        // Tango Service should automatically attempt to recover when it enters an invalid state.
        config.putBoolean(TangoConfig.KEY_BOOLEAN_AUTORECOVERY, true);
        return config;
    }

    /**
     * Set up the callback listeners for the Tango Service and obtain other parameters required
     * after Tango connection.
     * Listen to new Pose data.
     */
    private void startupTango() {
        // Lock configuration and connect to Tango.
        // Select coordinate frame pair.
        final ArrayList<TangoCoordinateFramePair> framePairs =
                new ArrayList<TangoCoordinateFramePair>();
        framePairs.add(new TangoCoordinateFramePair(
                TangoPoseData.COORDINATE_FRAME_AREA_DESCRIPTION,
                TangoPoseData.COORDINATE_FRAME_DEVICE));

        // Listen for new Tango data.
        mTango.connectListener(framePairs, new Tango.TangoUpdateCallback() {
            @Override
            public void onPoseAvailable(final TangoPoseData pose) {
                if(mPoseCallback != null)
                    mPoseCallback.onTangoPoseAvailable(pose);
            }

            @Override
            public void onXyzIjAvailable(TangoXyzIjData xyzIj) {
                // We are not using onXyzIjAvailable for this app.
            }

            @Override
            public void onPointCloudAvailable(final TangoPointCloudData pointCloud) {
                if(mPointCloudCallback != null)
                    mPointCloudCallback.onTangoPointCloudAvailable((pointCloud));
            }

            @Override
            public void onTangoEvent(final TangoEvent event) {
                // Ignoring TangoEvents.
            }

            @Override
            public void onFrameAvailable(int cameraId) {
                // We are not using onFrameAvailable for this application.
            }
        });
    }
}
