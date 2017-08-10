package de.julianloehr.tangocontroller;

import android.util.Log;

import com.google.atap.tangoservice.fois.FoiRequest;

import org.eclipse.paho.android.service.MqttAndroidClient;
import org.eclipse.paho.client.mqttv3.DisconnectedBufferOptions;
import org.eclipse.paho.client.mqttv3.IMqttActionListener;
import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.IMqttMessageListener;
import org.eclipse.paho.client.mqttv3.IMqttToken;
import org.eclipse.paho.client.mqttv3.MqttCallbackExtended;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;

public class MQTTWrapper {

    private MainActivity mContext;
    private MqttAndroidClient mqttAndroidClient;

    //final String serverUri = "tcp://iot.eclipse.org:1883";
    final String serverUri = "tcp://192.168.188.201:1883";

    final String UpdateTopic = "TangoController/Controller/Update";

    public MQTTWrapper(MainActivity Context) {
        mContext = Context;
    }

    public void Start()
    {
        Log.i(MainActivity.TAG, "Start MQTT");
        CreateMQTTClient();
        Connect();
    }

    private void CreateMQTTClient()
    {
        String clientId = MainActivity.TAG + System.currentTimeMillis();

        mqttAndroidClient = new MqttAndroidClient(mContext.getApplicationContext(), serverUri, clientId);
        mqttAndroidClient.setCallback(new MqttCallbackExtended() {
            @Override
            public void connectComplete(boolean reconnect, String serverURI) {
                Log.i(MainActivity.TAG, "Successfully connected to: " + serverURI);

                if (reconnect) {
                    //mContext.showsToastAndFinishOnUiThread("Reconnected to : " + serverURI);
                    // Because Clean Session is true, we need to re-subscribe
                    //subscribeToTopic();
                } else {
                   // mContext.showsToastAndFinishOnUiThread("Connected to: " + serverURI);
                }
            }

            @Override
            public void connectionLost(Throwable cause) {
                Log.i(MainActivity.TAG, "The Connection was lost.");
            }

            @Override
            public void messageArrived(String topic, MqttMessage message) throws Exception {
                Log.i(MainActivity.TAG, "Incoming message: " + new String(message.getPayload()));
            }

            @Override
            public void deliveryComplete(IMqttDeliveryToken token) {

            }
        });
    }

    private void Connect()
    {
        MqttConnectOptions mqttConnectOptions = new MqttConnectOptions();
        mqttConnectOptions.setAutomaticReconnect(true);
        mqttConnectOptions.setCleanSession(true);

        try {
            Log.i(MainActivity.TAG, "Connecting to " + serverUri);
            mqttAndroidClient.connect(mqttConnectOptions, null, new IMqttActionListener() {
                @Override
                public void onSuccess(IMqttToken asyncActionToken) {
                    //subscribeToTopic();

                    Log.i(MainActivity.TAG, "Successfully called Connect: " + serverUri);
                }

                @Override
                public void onFailure(IMqttToken asyncActionToken, Throwable exception) {
                    //mContext.showsToastAndFinishOnUiThread("Failed to connect to: " + serverUri);
                    Log.e(MainActivity.TAG, exception.getMessage(), exception);
                }
            });


        } catch (MqttException ex){
            mContext.showsToastAndFinishOnUiThread(ex.getMessage());
            Log.e(MainActivity.TAG, ex.getMessage(), ex);
        }
    }

    public void Stop()
    {
        Log.i(MainActivity.TAG, "Stop MQTT");

        if((mqttAndroidClient == null) || (!mqttAndroidClient.isConnected()))
            return;

        try {
            //Disconnect
            mqttAndroidClient.disconnect();
        } catch (MqttException ex){
            mContext.showsToastAndFinishOnUiThread(ex.getMessage());
            Log.e(MainActivity.TAG, ex.getMessage(), ex);
        }
    }

    public void SendUpdate(byte[] payload) {
        try {
            MqttMessage message = new MqttMessage();
            message.setPayload(payload);
            mqttAndroidClient.publish(UpdateTopic, message);

        } catch (MqttException ex) {
            mContext.showsToastAndFinishOnUiThread("Error Publishing: " + ex.getMessage());
            Log.e(MainActivity.TAG, ex.getMessage(), ex);
        }
    }
}
