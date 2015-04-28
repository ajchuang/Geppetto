import java.io.*;
import java.net.*;

import com.thalmic.myo.DeviceListener;
import com.thalmic.myo.Hub;
import com.thalmic.myo.Myo;
import com.thalmic.myo.Pose;

public class MrGeppettoMyo_Feedback implements Runnable {

    DataCollector m_dl;
    int m_port;

    public MrGeppettoMyo_Feedback (DataCollector dl, int p) {
        m_dl = dl;
        m_port = p;
    }

    public void run () {

        try {
            DatagramSocket serverSocket = new DatagramSocket (m_port); 
            byte[] inData = new byte[256];

            while (true) {
                DatagramPacket rcv = new DatagramPacket (inData, inData.length);
                serverSocket.receive (rcv);

                String in = new String (rcv.getData ());
                System.out.println (in);

                if (in == "vib") {
                    m_dl.vib ();
                }
            }
        } catch (Exception e) {
            System.err.println (e);
            e.printStackTrace ();
        }
    }
}
