package de.julianloehr.tangocontroller;

import android.os.Handler;
import android.os.HandlerThread;
import android.util.Log;

import com.google.atap.tango.mesh.TangoMesh;
import com.google.atap.tango.reconstruction.Tango3dReconstruction;
import com.google.atap.tango.reconstruction.Tango3dReconstructionConfig;
import com.google.atap.tangoservice.TangoCameraIntrinsics;
import com.google.atap.tangoservice.TangoInvalidException;
import com.google.atap.tangoservice.TangoPointCloudData;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.tango.support.TangoPointCloudManager;
import com.google.tango.support.TangoSupport;

import java.util.ArrayList;
import java.util.List;

public class MeshConstructor implements TangoWrapper.OnTangoPointCloudAvailableListener {
    public static final String TAG = MeshConstructor.class.getSimpleName();

    public interface OnTangoMeshesAvailableListener {
        void onTangoMeshesAvailable(final TangoMesh[] meshes);
    }

    private Tango3dReconstruction mTango3dReconstruction;
    private TangoPointCloudManager mPointCloudManager;
    private OnTangoMeshesAvailableListener mMeshCallback;

    private HandlerThread workerThread;
    private volatile Handler workerThreadHandler;
    private Runnable workerJob;

    public MeshConstructor(OnTangoMeshesAvailableListener meshCallback)
    {
        mMeshCallback = meshCallback;
        mTango3dReconstruction = new Tango3dReconstruction(new Tango3dReconstructionConfig());
        mPointCloudManager = new TangoPointCloudManager();

        workerThread = new HandlerThread("Mesh Constructor");
        workerThread.start();
        workerThreadHandler = new Handler(workerThread.getLooper());

        createWorkerJob();
    }

    public synchronized void setDepthCameraCalibration(TangoCameraIntrinsics calibration) {
        mTango3dReconstruction.setDepthCameraCalibration(calibration);
    }

    public void onTangoPointCloudAvailable(final TangoPointCloudData pointCloud)
    {
        if((pointCloud == null) || (pointCloud.points == null))
            return;

        mPointCloudManager.updatePointCloud(pointCloud);
        workerThreadHandler.removeCallbacksAndMessages(null);
        workerThreadHandler.post(workerJob);
    }

    private void createWorkerJob()
    {
        workerJob = new Runnable() {
            @Override
            public void run() {
                synchronized (MeshConstructor.this)
                {
                    TangoPointCloudData pointCloud = mPointCloudManager.getLatestPointCloud();
                    if (pointCloud == null) {
                        return;
                    }

                    TangoPoseData depthPose = null;
                    try {
                        depthPose = TangoSupport.getPoseAtTime(pointCloud.timestamp,
                                TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                                TangoPoseData.COORDINATE_FRAME_CAMERA_DEPTH,
                                TangoSupport.ENGINE_TANGO,
                                TangoSupport.ENGINE_TANGO,
                                TangoSupport.ROTATION_IGNORED);
                        if (depthPose.statusCode != TangoPoseData.POSE_VALID) {
                            Log.e(TAG, "Invalid Pose!");
                            return;
                        }
                    } catch (TangoInvalidException e) {
                        e.printStackTrace();
                        depthPose = null;
                    }
                    if (depthPose == null) {
                        Log.e(TAG, "No Pose data!");
                        return;
                    }
                    List<int[]> updatedIndices = mTango3dReconstruction.update(pointCloud, depthPose, null, null);

                    if (updatedIndices != null)
                    {
                        int indexCount = updatedIndices.size();
                        List<TangoMesh> meshes = new ArrayList<TangoMesh>(indexCount);

                        for (int i = 0; i < indexCount; ++i) {
                            TangoMesh mesh = mTango3dReconstruction.extractMeshSegment(updatedIndices.get(i));
                            if (mesh.numVertices > 0 && mesh.numFaces > 0) {
                                meshes.add(mesh);
                            }
                        }

                        TangoMesh[] meshArray = new TangoMesh[meshes.size()];
                        meshes.toArray(meshArray);
                        mMeshCallback.onTangoMeshesAvailable(meshArray);
                    }
                }
            }
        };
    }
}
