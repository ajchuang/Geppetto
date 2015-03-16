package com.geppetto.grasp.geppettoctrl;

import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.hardware.*;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import java.io.DataOutputStream;
import java.io.ObjectOutputStream;
import java.io.OutputStreamWriter;
import java.net.Socket;
import java.util.ArrayDeque;
import java.util.Iterator;


public class MainActivity extends ActionBarActivity
        implements SensorEventListener, View.OnClickListener {

    /* @lfred: data members */
    SensorManager mSensorManager;
    Sensor mAccMeter;
    Sensor mMagMeter;
    Sensor mGyroMeter;
    Sensor mRotMeter;

    /* init TCP connection */
    Socket m_socket;
    String m_serverIp;
    String m_serverPort;
    OutputStreamWriter m_outputStream;

    /* UI members */
    TextView m_dbgView;
    Button m_goButton;
    Button m_stopButton;
    Button m_backButton;

    /* internal data structure */
    ArrayDeque<DataSample> m_queue;

    /* log tag */
    final static String M_LOGTAG = "JH3478";
    final static int M_GYRO = 1;
    final static int M_CMD  = 2;
    final static int M_N_SAMPLES = 10;

    /* NW functions */
    boolean init_connection () {

        try {
            Log.v (M_LOGTAG, "Connect to " + m_serverIp + ":" + m_serverPort);
            //String hello = new String ("GYRO ");
            m_socket = new Socket (m_serverIp, Integer.parseInt (m_serverPort));
            m_outputStream = new OutputStreamWriter (m_socket.getOutputStream ());
            //m_outputStream.write (hello.toCharArray(), 0, hello.length());
            //m_outputStream.flush ();
        } catch (Exception e) {
            Log.v (M_LOGTAG, "failed to connect to the server");
            return false;
        }

        return true;
    }

    void shutdown_link () {

        try {
            m_outputStream.close ();
            m_socket.close ();
        } catch (Exception e) {
            Log.v (M_LOGTAG, "failed to close socket");
        }
    }

    void send_data (long ts, float x, float y, float z) {

        float x_sum = 0.0f, y_sum = 0.0f, z_sum = 0.0f;

        try {

            /* smooth the data set by using sliding window average */
            DataSample ds = new DataSample (ts, x, y, z);

            if (m_queue.size() == 10) {

                Iterator it = m_queue.iterator ();

                while (it.hasNext ()) {
                    DataSample t = (DataSample) it.next ();
                    x_sum += t.m_x;
                    y_sum += t.m_y;
                    z_sum += t.m_z;
                }

                m_queue.removeFirst ();
                m_queue.addLast (ds);
            } else {
                m_queue.addLast (ds);
                return;
            }

            String out =
                    "GO " + ts + " " +
                    x_sum/M_N_SAMPLES + " " +
                    y_sum/M_N_SAMPLES + " " +
                    z_sum/M_N_SAMPLES + " ";

            m_outputStream.write (out.toCharArray(), 0, out.length());
            m_outputStream.flush ();
        } catch (Exception e) {
            Log.v (M_LOGTAG, "failed to send data");
        }

    }

    void initSensor() {
        mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        mAccMeter = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        mGyroMeter = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        mRotMeter = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
    }

    void registerCallback() {
        mSensorManager.registerListener(this, mAccMeter, SensorManager.SENSOR_DELAY_UI);
        mSensorManager.registerListener(this, mGyroMeter, SensorManager.SENSOR_DELAY_UI);
        mSensorManager.registerListener(this, mRotMeter, SensorManager.SENSOR_DELAY_UI);
    }

    void dismissCallback() {
        mSensorManager.unregisterListener(this);
    }

    @Override
    protected void onCreate (Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        /* init the data */
        init_connection ();
        initSensor ();
        m_dbgView       = (TextView) findViewById(R.id.debugView);
        m_goButton      = (Button) findViewById(R.id.GoButton);
        m_stopButton    = (Button) findViewById(R.id.StopButton);
        m_backButton    = (Button) findViewById(R.id.BackButton);
    }

    @Override
    protected void onResume() {
        super.onResume();
        init_connection ();
        registerCallback ();
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_main, menu);
        return true;
    }

    @Override
    public void onPause() {
        super.onPause();
        shutdown_link ();
        dismissCallback ();
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            return true;
        }

        return super.onOptionsItemSelected(item);
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    }

    @Override
    public void onSensorChanged(SensorEvent event) {

        if (event.sensor == mRotMeter) {
            processRot (event);
        } else if (event.sensor == mGyroMeter) {
            processGyro (event);
        } else if (event.sensor == mAccMeter) {
            processAcc (event);
        } else {
            /* What !? */
        }
    }

    @Override
    public void onClick (View v) {

        if (v == m_goButton) {

        } else if (v == m_stopButton) {

        } else if (v == m_backButton) {

        } else {
            /* What !? */
        }
    }

    void processRot (SensorEvent event) {
        float axisX = event.values[0];
        float axisY = event.values[1];
        float axisZ = event.values[2];

        String out = "GO " + event.timestamp + " " + axisX + " " + axisY + " " + axisZ + " ";
        m_dbgView.setText (out);
        send_data (event.timestamp, axisX, axisY, axisZ);
    }

    void processGyro (SensorEvent event) {
/*
        float axisX = event.values[0];
        float axisY = event.values[1];
        float axisZ = event.values[2];

        String out = "GO " + event.timestamp + " " + axisX + " " + axisY + " " + axisZ + " ";
        m_dbgView.setText (out);
        send_data (event.timestamp, axisX, axisY, axisZ);
*/
    }

    void processAcc (SensorEvent event) {

    }
}