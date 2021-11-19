package lecture4;

import ev3dev.actuators.Sound;
import ev3dev.actuators.lego.motors.BaseRegulatedMotor;
import ev3dev.actuators.lego.motors.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Waypoint;
import lejos.robotics.pathfinding.Path;
import lejos.utility.Delay;

public class PathFinding {
	public static void main(String[] args) throws Exception {
		BaseRegulatedMotor mL = new EV3LargeRegulatedMotor(MotorPort.A);
		BaseRegulatedMotor mR = new EV3LargeRegulatedMotor(MotorPort.B);

		EV3Pilot pilot = new EV3Pilot(60, 58, mL, mR);
		PoseProvider poseP = new OdometryPoseProvider(pilot);

		Navigator nav = new Navigator(pilot, poseP);
		float[] samples = new float[1];
		Thread t = new Watcher(nav, samples); 
		t.start();
		Path route = new Path();
		route.add(new Waypoint(100, 0));
		route.add(new Waypoint(100, 100));
		route.add(new Waypoint(0, 100));
		route.add(new Waypoint(0, 0));
		nav.followPath(route); // followPath returns immediately, so
		nav.waitForStop(); // we wait for the Navigator to stop!


		while (!nav.pathCompleted()) {
			while (samples[0] < 0.25f) { // Start again when obstacle > 25cm 
				Sound.getInstance().beep(); // TODO: fix dave's slides 
				Delay.msDelay(500);
			}
			nav.followPath();
			nav.waitForStop();
		}

	}
}
