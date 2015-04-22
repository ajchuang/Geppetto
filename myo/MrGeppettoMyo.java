import com.thalmic.myo.DeviceListener;
import com.thalmic.myo.Hub;
import com.thalmic.myo.Myo;

public class MrGeppettoMyo {

    static void log (String s) { System.err.println (s); }
	
    public static void main(String[] args) {

        try {
            /* init the myo hub */
			Hub hub = new Hub ("com.example.hello-myo");
            Myo device = null;
            int wCounter = 0;
            
            for (int i=0; i<10 && device == null; ++i) {
			    log ("Attempting to find a Myo...");
			    device = hub.waitForMyo (5000);
            }

            if (device == null) {
                log ("Failed to find a Myo.");
                return;
            }

			DeviceListener dataCollector = new DataCollector();
			hub.addListener (dataCollector);

			while (true) {
				hub.run (1000 / 200);
				log ("[Status] " + dataCollector);
			}
		} catch (Exception e) {
			e.printStackTrace();
			System.exit(1);
		}
	}
}
