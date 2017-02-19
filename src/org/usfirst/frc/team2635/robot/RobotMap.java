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
	public static int SHOOTER_FIRE_CONTROL_FORWARD = 2;
	public static int SHOOTER_FIRE_CONTROL_BACKWARDS = 3;
	
	public static int PICKUP_BALL = 1;
	
	public static int ROPE_CLIMBER_1 = 1;
	public static int ROPE_CLIMBER_2 = 2;
	
	public static int DELIVER_GEAR_FORWARD = 0;
	public static int DELIVER_GEAR_BACKWARDS = 1;
	
	public static int ULTRASONIC_LEFT = 1;
	public static int ULTRASONIC_RIGHT = 2;
	
	public static int JOYSTICK_LEFT = 0;
	public static int JOYSTICK_RIGHT = 1;
	
	public static int LEFT_DRIVE_Y = 1;
	public static int RIGHT_DRIVE_Y = 1;
	
	public static int BUTTON_REV_UP = 1;
	public static int BUTTON_FIRE = 1;
	public static int BUTTON_FEED_IN = 2;
	public static int BUTTON_FEED_OUT = 3;
	public static int BUTTON_CLIMB_UP = 3;
	public static int BUTTON_CLIMB_DOWN = 2;
	public static int BUTTON_DELIVER = 5;
	public static int BUTTON_AIM_CAMERA = 9;
	public static int BUTTON_MOTION_MAGIC = 8;
	
	public static double WHEEL_RADIUS_INCHES = 1.45; 
	public static double WHEEL_SEPARATION_INCHES = 19.0; 
	public static double AIM_P = 0.0;
	public static double AIM_I = 0.0;
	public static double AIM_D = 0.0;
	

	
	public static double MOTION_MAGIC_F = 1.5345;
    public static double MOTION_MAGIC_P = 10;
    public static double MOTION_MAGIC_I = 0.1; 
    public static double MOTION_MAGIC_D = 0;
	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
}
