package org.usfirst.frc.team2635.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;
	public static int DRIVE_RIGHT_FRONT = 9;
	public static int DRIVE_RIGHT_BACK = 6;
	public static int DRIVE_LEFT_FRONT = 4;
	public static int DRIVE_LEFT_BACK = 12;
	
	public static int SHOOTER_BALL_FLYWHEEL = 1;
	public static int SHOOTER_AGITATOR = 2;
	public static int SHOOTER_FIRE_CONTROL_FORWARD = 3;
	public static int SHOOTER_FIRE_CONTROL_BACKWARDS = 4;
	
	public static int PICKUP_BALL = 1;
	
	public static int ROPE_CLIMBER_1 = 1;
	public static int ROPE_CLIMBER_2 = 2;
	
	public static int DELIVER_GEAR_FORWARD = 1;
	public static int DELIVER_GEAR_BACKWARDS = 2;
	
	public static int ULTRASONIC_LEFT = 1;
	public static int ULTRASONIC_RIGHT = 2;
	
	public static int JOYSTICK_LEFT = 0;
	public static int JOYSTICK_RIGHT = 1;
	
	public static int LEFT_DRIVE_Y = 1;
	public static int RIGHT_DRIVE_Y = 1;
	
	
	
	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
}
