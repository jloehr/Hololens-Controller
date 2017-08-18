package de.julianloehr.tangocontroller;

import android.app.Activity;
import android.widget.Toast;

public class ShowsToastAndFinishOnUiThread implements TangoWrapper.ShowsToastAndFinishOnUiThreadInterface {

    private Activity mActivity;

    public ShowsToastAndFinishOnUiThread(Activity activity)
    {
        mActivity = activity;
    }

    public void showsToastAndFinishOnUiThread(final int resId) {
        showsToastAndFinishOnUiThread(mActivity.getString(resId));
    }

    public void showsToastAndFinishOnUiThread(final String text) {
        mActivity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(mActivity,
                        text, Toast.LENGTH_LONG).show();
                mActivity.finish();
            }
        });
    }
}
