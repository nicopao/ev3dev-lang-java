import ev3dev.sensors.Button;
import lejos.robotics.SampleProvider;


public class MinMaxFilter implements SampleProvider {
	private SampleProvider sp;
	private float range = 1.0f;
	private float offset = 0.0f;

	public MinMaxFilter(SampleProvider sp) { // The actual sensor object will be used to get un-normalised values.
		this.sp = sp;
	}

	public void calibrate() { // This function is called once and then it keeps getting values until ENTER is
								// pressed
		float[] samples = new float[1];
		float highValue = 0.0f;
		float lowValue = 1.0f;
		while (!Button.ENTER.isDown()) {
			sp.fetchSample(samples, 0);
			highValue = Math.max(highValue, samples[0]);
			lowValue = Math.min(lowValue, samples[0]);
		}
		offset = lowValue;
		range = highValue - lowValue; // Before returning it stores the range and offset values for future use.
	}

	public int sampleSize() {
		return 1;
	}

	public void fetchSample(float[] sample, int index) {
		sp.fetchSample(sample, index); // delegate the fetchSample call to the actual sensor
		sample[index] = (sample[index] - offset) / range; // Adjust the result and put it into the
															// returned array
		return;
	}
}