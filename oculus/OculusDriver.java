//package no.hials.vr;

import com.google.gson.Gson;
import com.oculusvr.capi.Hmd;

import static com.oculusvr.capi.OvrLibrary.ovrTrackingCaps.*;

import java.util.*;
import java.io.*;
import java.net.*;

import com.oculusvr.capi.OvrQuaternionf;
import com.oculusvr.capi.OvrVector3f;
import com.oculusvr.capi.TrackingState;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.UnknownHostException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Scanner;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;

import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;

import javax.swing.JFrame;
import javax.swing.JTextField;

/**
 * WebSocket server that provides tracking info Dependencies are: JOVR, GSON and
 * Java_WebSockets
 *
 * @author Lars Ivar Hatledal
 */

public class OculusWS {

    private static final int port = 11111;
    private static final String ip = "128.59.22.124";
    private static long id = 0;
    private static SensorData latestData = new SensorData(0, 0, 0, 0, 0, 0, 0, 0);
    private static boolean run = true;
    private static boolean oculusMode = true;
    private static boolean socketMode = true;
    private static Thread sensorThread = null;
    private static Thread sendThread = null;
    private static Thread keyListeningThread = null;

    /**
     * Program starting point
     *
     * @param args the command line arguments
     * @throws java.net.UnknownHostException
     */
    public static void main(String[] args) throws UnknownHostException {
        Hmd.initialize();
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new IllegalStateException(e);
        }
        
        Hmd hmd = Hmd.create(0);

        if (hmd == null) {
            // throw new IllegalStateException("Unable to initialize HMD");
            System.out.println("Non-Oculus mode started.");
            oculusMode = false;
        }

        if (oculusMode) {
            hmd.configureTracking(ovrTrackingCap_Orientation
                    | ovrTrackingCap_MagYawCorrection
                    | ovrTrackingCap_Position, 0);

            sensorThread = new Thread(new SensorFetcher(hmd));
            sensorThread.start();
        }

		try {
            sendThread = new SendThread(ip, port);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
        sendThread.start();

        keyListeningThread = new KeyListeningThread();
        keyListeningThread.start();

        System.out.println("Press 'q' to quit..");
        System.out.println("");
        Scanner sc = new Scanner(System.in);
        while (run) {
            if (sc.nextLine().trim().equals("q")) {
                System.out.println("ouch");
                run = false;
            }
        }
        
        if (oculusMode) {
            try {
                sensorThread.join();
            } catch (InterruptedException ex) {
                Logger.getLogger(OculusWS.class.getName()).log(Level.SEVERE, null, ex);
            }
            hmd.destroy();
            Hmd.shutdown();
        }

        System.exit(-1);

    }


    public static class KeyListeningThread extends Thread {

        public void run() {
            JFrame aWindow = new JFrame("Key Active Area");
            aWindow.setBounds(50, 100, 300, 300);
            aWindow.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            JTextField typingArea = new JTextField(20);
            typingArea.addKeyListener(new KeyListener() {

                /** Handle the key pressed event from the text field. */
                @Override
                public void keyPressed(KeyEvent e) {
                    switch (e.getKeyCode()) {
                        // up
                        case 38:
                            latestData.px = latestData.px + 0.1;
                            break;
                        // down
                        case 40:
                            latestData.px = latestData.px - 0.1;
                            break;
                        // left
                        case 37:
                            latestData.py = latestData.py - 0.1;
                            break;
                        // right
                        case 39:
                            latestData.py = latestData.py + 0.1;
                            break;
                    }
                }

				@Override
				public void keyTyped(KeyEvent e) {
					// TODO Auto-generated method stub
				}

				@Override
				public void keyReleased(KeyEvent e) {
					// TODO Auto-generated method stub
					
				}

            });
            aWindow.add(typingArea);
            aWindow.setVisible(true);
        }
    }

    public static class SendThread extends Thread {

        public Socket socket;
        public String ip;
        public int port;
        public PrintWriter out;

        public SendThread(String ip, int port) throws UnknownHostException, IOException {
            this.ip = ip;
            this.port = port;
            try {
                this.socket = new Socket(ip, port);
            } catch (ConnectException e) {
                socketMode = false;
                System.out.println("Local mode started");
            }
        }

        public void run() {

            try {
                if (socketMode) {
                    out = new PrintWriter(socket.getOutputStream());
                }
                // Writer writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream("filename.txt"), "utf-8"));
                // sin = new BufferedReader(new InputStreamReader(System.in));
                int count = 0;
                while (true) {
                    if (socketMode){
                        out.println(latestData.dump());    
                        out.flush();
                    }
                    /*
                    writer.write("\n");
                    writer.write(latestData.dump());
                    writer.flush();
                    */
                    // System.out.println(count++);
                    System.out.print(latestData.toString());
                    Thread.sleep(200);
                    System.out.print("\r");
                }
            } catch (IOException e) {
            System.err.println("IOException: " + e);
            } catch (InterruptedException e) {
				e.printStackTrace();
			}   
        }
    }


    private static class SensorFetcher implements Runnable {

        private final Hmd hmd;

        public SensorFetcher(Hmd hmd) {
            this.hmd = hmd;
        }

        @Override
        public void run() {
            while (run) {
                TrackingState sensorState = hmd.getSensorState(Hmd.getTimeInSeconds());

                OvrVector3f pos = sensorState.HeadPose.Pose.Position;
                OvrQuaternionf quat = sensorState.HeadPose.Pose.Orientation;

                double px = pos.x;
                double py = pos.y;
                double pz = pos.z;

                double qx = quat.x;
                double qy = quat.y;
                double qz = quat.z;
                double qw = quat.w;

                latestData = new SensorData(id++, px, py, pz, qx, qy, qz, qw);
                //System.out.println(latestData);

                try {
                    Thread.sleep(1);
                } catch (InterruptedException ex) {
                    Logger.getLogger(OculusWS.class.getName()).log(Level.SEVERE, null, ex);
                }
            }
        }
    }

    private static class SensorData {

        private final long id;
        public double px, py, pz, qx, qy, qz, qw;

        public SensorData(long id, double px, double py, double pz, double qx, double qy, double qz, double qw) {
            this.id = id;
            this.px = px;
            this.py = py;
            this.pz = pz;
            this.qx = qx;
            this.qy = qy;
            this.qz = qz;
            this.qw = qw;
        }

        public String dump() {
            return String.format("s  %.3f  %.3f  %.3f  %.3f  %.3f  %.3f  %.3f", px, py, pz, qx, qy, qz, qw);
        }
        public long getId() {
            return id;
        }

        public double[] asArray() {
            return new double[]{id, px, py, pz, qx, qy, qz, qw};
        }

        @Override
        public String toString() {
            return String.format("Position: %.3f  %.3f  %.3f | Quat:  %.3f  %.3f  %.3f  %.3f", px, py, pz, qx, qy, qz, qw);
        }
    }
}
