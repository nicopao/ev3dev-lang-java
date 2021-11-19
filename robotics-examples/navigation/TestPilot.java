package lecture4;

import ev3dev.actuators.lego.motors.BaseRegulatedMotor;
import ev3dev.actuators.lego.motors.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;

public class TestPilot {

	public static void main(String[] args) {

		float diameter = Float.parseFloat(args[0]);
		float trackwidth = Float.parseFloat(args[1]);
		boolean reverse = Boolean.parseBoolean(args[2]);

		BaseRegulatedMotor mLeft = new EV3LargeRegulatedMotor(MotorPort.A);
		BaseRegulatedMotor mRight = new EV3LargeRegulatedMotor(MotorPort.B);
//		mLeft.synchronizeWith(new BaseRegulatedMotor[] {mRight});
//		EV3Pilot pilot = new EV3Pilot(60, 58, mLeft, mRight); // reverse is false by default
//		EV3Pilot pilot = new EV3Pilot(56, 120, mLeft, mRight);
		EV3Pilot pilot = new EV3Pilot(diameter, trackwidth, mLeft, mRight, reverse);
		for (int side = 0; side < 4; side++) {
			pilot.travel(300);
			pilot.rotate(90);
		}

	}

}
