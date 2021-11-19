package lecture4;

import ev3dev.actuators.lego.motors.BaseRegulatedMotor;
import ev3dev.actuators.lego.motors.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Waypoint;
import lejos.robotics.pathfinding.Path;

public class PathFollowing {
	public static void main(String[] args) throws Exception {
		BaseRegulatedMotor mL = new EV3LargeRegulatedMotor(MotorPort.A);
		BaseRegulatedMotor mR = new EV3LargeRegulatedMotor(MotorPort.B);

		EV3Pilot pilot = new EV3Pilot(60, 58, mL, mR);
		PoseProvider poseP = new OdometryPoseProvider(pilot);

		Navigator navigator = new Navigator(pilot, poseP);
		Path route = new Path();
		route.add(new Waypoint(100, 0));
		route.add(new Waypoint(100, 100));
		route.add(new Waypoint(0, 100));
		route.add(new Waypoint(0, 0));
		navigator.followPath(route); // followPath returns immediately, so
		navigator.waitForStop(); // we wait for the Navigator to stop!

	}
}

