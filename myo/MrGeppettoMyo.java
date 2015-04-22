import java.io.*;
import java.net.*;

import com.thalmic.myo.DeviceListener;
import com.thalmic.myo.Hub;
import com.thalmic.myo.Myo;

public class MrGeppettoMyo {

    int m_port;
    String m_host;
    Socket m_socket; //= new Socket ("localhost", 6789);
    DataOutputStream m_outStream;

    public MrGeppetto (String host, int port) {
        
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
            log (e);
            e.printStackTrace ();
        }

        return false;
    }

    public boolean send (String) {
        
        try {

            /* send over TCP */
            
            return true;

        } catch (Exception e) {
            e.printStackTrace ();
        }

        return false;
    }

    public void deinit () {
        try {
            m_outStream.close ();
            m_socket.close ();
        } catch (Exception e) {
            log (e);
            e.printStackTrace ();
        }
    }

    public static void log (String s) { System.err.println (s); }
    public static void err (String s) { "[Error] " + System.err.println (s); }

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
			    device = hub.waitForMyo (5000);
            }

            if (device == null) {
                err ("Unable to find a Myo.");
                return;
            }

            /* attach the device listener */
			DeviceListener dataCollector = new DataCollector();
			hub.addListener (dataCollector);

            /* establish the connection */
            MrGeppetto mg = new MrGeppetto (host, port);
            
            if (mg.connect () == false) {
                err ("Failed to connect to the host");
                return false;
            }

			while (true) {
                /* report per 0.5 seconds */
				hub.run (1000 / 200);
				log ("[Status] " + dataCollector);
			}
		} catch (Exception e) {
			e.printStackTrace ();
			System.exit (1);
		}
	}
}
