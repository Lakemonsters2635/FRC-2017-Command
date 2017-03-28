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
	public static final int DRIVE_RIGHT_FRONT = 1;
	public static final int DRIVE_RIGHT_BACK = 2;
	public static final int DRIVE_LEFT_FRONT = 3;
	public static final int DRIVE_LEFT_BACK = 4;
	
	public static final int SHOOTER_BALL_FLYWHEEL = 5;
	public static final int SHOOTER_AGITATOR = 6;
	public static final int SHOOTER_FIRE_CONTROL_FORWARD = 3;
	public static final int SHOOTER_FIRE_CONTROL_BACKWARDS = 2;
	
	//TODO: Change these to valid channel numbers
	public static final int SHOOTER_AGITATE_FORWARD = 4;
	public static final int SHOOTER_AGITATE_REVERSE = 5;

	public static final double SHOOTER_AGITATE_TIME = 1.00;
	
	public static final int PICKUP_BALL = 7;
	
	public static final int ROPE_CLIMBER_1 = 8;
	public static final int ROPE_CLIMBER_2 = 9;
	
	public static final int DELIVER_GEAR_FORWARD = 1;
	public static final int DELIVER_GEAR_BACKWARDS = 0;
	
	public static final int ULTRASONIC_LEFT = 1;
	public static final int ULTRASONIC_RIGHT = 2;

	
	public static final int JOYSTICK_LEFT = 0;
	public static final int JOYSTICK_RIGHT = 1;
	
	public static final int LEFT_DRIVE_Y = 1;
	public static final int LEFT_DRIVE_Z = 2;
	public static final int RIGHT_DRIVE_Y = 1;
	public static final int RIGHT_DRIVE_Z = 2;
	
	public static final int BUTTON_REV_UP = 6; //FHE TODO: TALK TO DRIVE TEAM
	public static final int BUTTON_FIRE = 11; //FHE TODO: TALK TO DRIVE TEAM
	public static final int BUTTON_FEED_IN = 2;
	public static final int BUTTON_FEED_OUT = 3;
	public static final int BUTTON_CLIMB_UP = 2;
	public static final int BUTTON_CLIMB_DOWN = 3;
	public static final int BUTTON_DELIVER = 5;
	public static final int BUTTON_AIM_CAMERA = 9;
	public static final int BUTTON_MOTION_MAGIC = 8;
	public static final int BUTTON_NAVX_ROTATE = 8;
	public static final int BUTTON_AIM = 7;
	public static final int BUTTON_NAVX_GET_ANGLE = 10;
	public static final int BUTTON_GEAR_AUTODOC = 1;
	public static final int BUTTON_DRIVE_STRAIGHT = 1;
	
	public static final double WHEEL_RADIUS_INCHES = 3.0  * 1.0; 
	public static final double WHEEL_SEPARATION_INCHES = 24.5;
	
	public static final double BUMPER_TO_SONAR_DISTANCE = 4.0; //Distance from Outside of bumper to front-face of sonar.
	
	public static final double AIM_P = 10.0;
	public static final double AIM_I = 0.0;
	public static final double AIM_D = 0.0;
	
	public static final int TELEOP_AGITATOR_LOOP_COUNT = 2;
	
	public static final double MOTION_MAGIC_F = 1.5345;
    public static final double MOTION_MAGIC_P = 10;
    public static final double MOTION_MAGIC_I = 0.0004; 
    public static final double MOTION_MAGIC_D = 0;
    
	public static final double DRIVE_STRAIGHT_MOTION_MAGIC_F = 1.5;
    //public static double DRIVE_STRAIGHT_MOTION_MAGIC_P = .5;
	public static final double DRIVE_STRAIGHT_MOTION_MAGIC_P = 10;
    public static final double DRIVE_STRAIGHT_MOTION_MAGIC_I =  0.0004; 
    public static final double DRIVE_STRAIGHT_MOTION_MAGIC_D = 0;
    
    
    public static final double DRIVE_STRAIGHT_TELEOP_MOTION_MAGIC_F = 1.5;
	public static final double DRIVE_STRAIGHT_TELEOP_MOTION_MAGIC_P = 1;
    public static final double DRIVE_STRAIGHT_TELEOP_MOTION_MAGIC_I =  0.0; 
    public static final double DRIVE_STRAIGHT_TELEOP_MOTION_MAGIC_D = 0;
	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
    
    public static final int LEFT_ULTRASONIC_CHANNEL = 1;
    public static final int RIGHT_ULTRASONIC_CHANNEL = 2;
    
    public static final int VISION_LIGHT_CHANNEL = 7;

    public static final boolean DEBUG_DETAIL = true;
    
 
    public static final double GEAR_DELIVERY_TIMEOUT = 0.5;  // units are seconds

}
