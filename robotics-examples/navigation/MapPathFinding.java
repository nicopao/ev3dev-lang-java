package lecture4;

import ev3dev.actuators.lego.motors.BaseRegulatedMotor;
import ev3dev.actuators.lego.motors.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.geometry.Line;
import lejos.robotics.geometry.Rectangle;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.mapping.LineMap;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Pose;
import lejos.robotics.navigation.Waypoint;
import lejos.robotics.pathfinding.Path;
import lejos.robotics.pathfinding.PathFinder;
import lejos.robotics.pathfinding.ShortestPathFinder;

public class MapPathFinding {
	public static void main(String[] args) throws Exception {
		BaseRegulatedMotor mL = new EV3LargeRegulatedMotor(MotorPort.A);
		BaseRegulatedMotor mR = new EV3LargeRegulatedMotor(MotorPort.B);

		EV3Pilot pilot = new EV3Pilot(60, 58, mL, mR);
		PoseProvider poseP = new OdometryPoseProvider(pilot);

		Navigator nav = new Navigator(pilot, poseP);
		
		Line [] lines = new Line[4];
		lines [0] = new Line(-20f, 20f, 100f, 20f);
		lines [1] = new Line(-20f, 40f, 20f, 40f);
		lines [2] = new Line(-20f, 60f, 20f, 60f);
		lines [3] = new Line(-20f, 80f, 20f, 80f);
		Rectangle bounds = new Rectangle(-50, -50, 250, 250); 
		LineMap myMap = new LineMap(lines, bounds); 
		PathFinder pf = new ShortestPathFinder(myMap);
		Path route = pf.findRoute(new Pose(0,0,0), new Waypoint(0, 100)); 

		nav.followPath(route); // followPath returns immediately, so
		nav.waitForStop(); // we wait for the Navigator to stop!



	}
}