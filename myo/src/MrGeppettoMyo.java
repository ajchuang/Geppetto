import java.io.*;
import java.net.*;

import com.thalmic.myo.DeviceListener;
import com.thalmic.myo.Hub;
import com.thalmic.myo.Myo;
import com.thalmic.myo.Pose;

public class MrGeppettoMyo {

    final static int M_MONITOR_INTR = 500;
    final static int M_WAIT_INTR    = 5000;

    int m_port;
    String m_host;
    Socket m_socket;
    DataOutputStream m_outStream;

    public MrGeppettoMyo (String host, int port) {
        
        m_host = host;
        m_port = port;
        
        m_socket = null;
        m_outStream = null;
    }

    public boolean connect (String host, int port) {

        try {
            m_socket = new Socket (host, port);
            m_outStream = new DataOutputStream (m_socket.getOutputStream ()); 
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
            m_outStream.writeChars (info);
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

        if (args.length != 2) {
            err ("Incorrect input format");
            log ("[Usage] java MrGeppetto [host] [port]");
            return;
        }

        String host = args[0];
        int port = Integer.parseInt (args[1]);

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

            /* establish the connection */
            MrGeppettoMyo mg = new MrGeppettoMyo (host, port);
            
            if (mg.connect (host, port) == false) {
                err ("Failed to connect to the host");
                return;
            }

			while (true) {
                /* report per 0.5 seconds */
				hub.run (M_MONITOR_INTR);
                
                /* send data only when connected */
                if (dc.isConnected ()) {

                    String newPose = dc.getPose ();
                    String out = "GO " + newPose + " " + dc.getRoll ();

                    if (mg.send (out)) {
                        //log ("[Status] " + dc.toString ());
                    } else {
                        err ("Network failure - Mr.Geppetto feels sorry.");
                        break;
                    }

                    /* do recording */
                    if (dc.isRecording ()) {
                        long offset = dc.getRecTimeOffset ();
                        MrGeppettoMyo.log ("[Rec] " + out + "@" + offset);
                    }
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
