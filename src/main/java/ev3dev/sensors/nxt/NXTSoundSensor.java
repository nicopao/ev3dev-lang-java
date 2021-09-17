package ev3dev.sensors.nxt;

import ev3dev.sensors.BaseSensor;
import ev3dev.sensors.GenericMode;
import ev3dev.utils.Sysfs;
import java.io.File;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;

public class NXTSoundSensor extends BaseSensor {

    private static final String LEGO_NXT_SOUND = "lego_nxt_sound";
    private static final String DBMODE = "NXT-SOUND-DB";
    private static final String DBAMODE = "NXT-SOUND-DBA";

    /**
     * Constructor
     *
     * @param port Port name 
     */
    public NXTSoundSensor(Port port) {

        super(port, LEGO_I2C, LEGO_NXT_SOUND);
    
        setModes(new SensorMode[] {
            new InternalMode(this.PATH_DEVICE, 1, "DB"),
            new InternalMode(this.PATH_DEVICE, 1, "DBA")
        });

    }
  
    public SampleProvider getDBMode() {
        switchMode(DBMODE, SWITCH_DELAY);
        return getMode(0);
    }
  
    public SampleProvider getDBAMode() {
        switchMode(DBAMODE, SWITCH_DELAY);
        return getMode(1);
    }
  
    private class InternalMode extends GenericMode {
        private File devicePath;

        public InternalMode(File pathDevice, int sampleSize, String modeName) {
            super(pathDevice, sampleSize, modeName);
            this.devicePath = pathDevice;
        }

    
        @Override
        public void fetchSample(float[] sample, int offset) {
            float reading = Sysfs.readFloat(this.devicePath + "/" + "value0");
            reading = reading / 10.0f;
            sample[offset] = reading;
        }

    }

}


