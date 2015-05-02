import java.io.*;
import java.net.*;

import com.thalmic.myo.DeviceListener;
import com.thalmic.myo.Hub;
import com.thalmic.myo.Myo;
import com.thalmic.myo.Pose;

public class MrGeppettoMyo {

    final static int M_MONITOR_INTR = 50;
    final static int M_WAIT_INTR    = 5000;

    int m_port;
    String m_host;
    Socket m_socket;
    PrintWriter m_outStream;

    int m_rPort;
    String m_rHost;
    Socket m_rSocket;
    PrintWriter m_rStream;

    public MrGeppettoMyo (String host, int port, String rhost, int rport) {
        
        m_host = host;
        m_port = port;
        m_socket = null;
        m_outStream = null;

        m_rPort = rport;
        m_rHost = rhost;
        m_rSocket = null;
        m_rStream = null;
    }

    public boolean connect () {

        try {
            m_socket = new Socket (m_host, m_port);
            m_outStream = new PrintWriter (m_socket.getOutputStream ()); 
            
            m_rSocket = new Socket (m_rHost, m_rPort);
            m_rStream = new PrintWriter (m_rSocket.getOutputStream ()); 
            
            return true;
        } catch (Exception e) {
            MrGeppettoMyo.err (e.toString ());
            e.printStackTrace ();
        }

        return false;
    }

    public boolean sendRoll (String roll) {
    
        try {
            m_rStream.print (roll);
            m_rStream.flush ();
            return true;
        } catch (Exception e) {
            MrGeppettoMyo.err (e.toString ());
            e.printStackTrace ();
        }

        return false;
    }

    public boolean send (String info) {
        
        try {

            /* send over TCP */
            m_outStream.print (info);
            m_outStream.flush ();
            return true;

        } catch (Exception e) {
            MrGeppettoMyo.err (e.toString ());
            e.printStackTrace ();
        }

        return false;
    }

    public void deinit () {
        try {
            m_outStream.close ();
            m_socket.close ();
        } catch (Exception e) {
            MrGeppettoMyo.err (e.toString ());
            e.printStackTrace ();
        }
    }

    public static void log (String s) { System.err.println (s); }
    public static void err (String s) { System.err.println ("[Error] " + s); }

    public static void main (String[] args) {

        if (args.length != 4) {
            err ("Incorrect input format: " + args.length);
            log ("[Usage] java MrGeppetto [host] [port] [kinect_host] [kinect_port]");
            return;
        }

        String host = args[0];
        int port = Integer.parseInt (args[1]);
        String rhost = args[2];
        int rport = Integer.parseInt (args[3]);

        try {
            /* init the myo hub */
			Hub hub = new Hub ("com.example.MrGeppetto");
            Myo device = null;
            int wCounter = 0;
            
            for (int i=0; i<10 && device == null; ++i) {
			    log ("Attempting to find a Myo...");
			    device = hub.waitForMyo (M_WAIT_INTR);
            }

            if (device == null) {
                err ("Unable to find a Myo armband.");
                return;
            }

            /* attach the device listener */
			DataCollector dc = new DataCollector ();
			hub.addListener ((DeviceListener)dc);

            /* starting vib server, and start */
            MrGeppettoMyo_Feedback fb = new MrGeppettoMyo_Feedback (dc, 7777);
            new Thread (fb).start ();

            /* establish the connection */
            MrGeppettoMyo mg = new MrGeppettoMyo (host, port, rhost, rport);
            
            if (mg.connect () == false) {
                err ("Failed to connect to the host");
                return;
            }

            boolean isRec = false;
            String curPose = null;
            int curRoll = -256;

			while (true) {
                /* report per 0.5 seconds */
				hub.run (M_MONITOR_INTR);
                
                /* send data only when connected */
                if (!dc.isConnected ())
                    continue;

                /* do recording */
                if (dc.isRecording ()) {
                    if (isRec == false) {
                        /* send start */
                        System.out.println ("[MYO] start rec");
                        mg.send ("START");
                        isRec = true;
                    }
                } else {
                    if (isRec == true) {
                        System.out.println ("[MYO] stop rec");
                        mg.send ("END");
                        isRec = false;
                    }
			    }

                /* send data */
                String newPose = dc.getPose ();
                int newRoll = dc.getRoll ();

                String out = "GO " + newPose + " " + (newRoll / 180.0 * Math.PI) + " ";
                String rol = "RL " + (newRoll / 180.0 * Math.PI) + " ";

                /* skip if no change */
                if (newPose.equals (curPose) && newRoll == curRoll)
                    continue;
                else {
                    curPose = newPose;
                    curRoll = newRoll;
                }

                if (mg.send (out) && mg.sendRoll (rol)) {
                    log ("[Status] " + out);
                } else {
                    err ("Network failure - Mr.Geppetto feels sorry.");
                    break;
                } 
            }

            err ("Bad thing happens");
            return;

		} catch (Exception e) {
			MrGeppettoMyo.err ("Ooops: Mr.Geppetto is sick - ");
            e.printStackTrace ();
			System.exit (1);
		}
	}
}
