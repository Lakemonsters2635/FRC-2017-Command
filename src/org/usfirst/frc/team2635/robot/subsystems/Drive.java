package org.usfirst.frc.team2635.robot.subsystems;

import java.util.function.Consumer;
import java.util.function.Function;

import org.usfirst.frc.team2635.robot.RobotMap;
import org.usfirst.frc.team2635.robot.commands.DriveTeleop;
import org.usfirst.frc.team2635.robot.model.DriveParameters;
import org.usfirst.frc.team2635.robot.model.Navx;
import org.usfirst.frc.team2635.robot.model.MotionParameters;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * The drive chassis
 */
public class Drive extends Subsystem {
	public static final double ANGLE_ERROR_TOLERANCE = 1;
	public static final double ROTATE_ERROR_TOLERANCE = 0.01;
	public static final double DRIVE_ERROR_TOLERANCE = 0.03;

	public double maxAcceleration;
	public double maxVelocity;
	public double leftWheelRotations;
	public double rightWheelRotations;

	boolean enableTankDriveWithEncoders;

	public double currentHeadingOffset = 0;
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	CANTalon rightFront;
	CANTalon rightBack;
	CANTalon leftFront;
	CANTalon leftBack;
	
	DriveTeleop teleopCommand;
	
	public double errNavxDrive;
	
	Navx navx = new Navx();

	public Drive(double maxAcceleration, double maxVelocity, double leftWheelRotations, double rightWheelRotations, boolean enableTankDriveWithEncoders) {
		this.maxAcceleration = maxAcceleration;
		this.maxVelocity = maxVelocity;
		this.leftWheelRotations = leftWheelRotations;
		this.rightWheelRotations = rightWheelRotations;
		this.enableTankDriveWithEncoders = enableTankDriveWithEncoders;
	}

	class NavxUnwrappedAnglePIDSource implements PIDSource {
		Navx navx;

		public NavxUnwrappedAnglePIDSource(Navx navx) {
			this.navx = navx;
		}

		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {
			// TODO Auto-generated method stub
		}

		@Override
		public PIDSourceType getPIDSourceType() {
			return PIDSourceType.kDisplacement;
		}

		@Override
		public double pidGet() {
			return navx.getAngle();
		}
	}

	class DrivePIDOutput implements PIDOutput {
		public DrivePIDOutput(RobotDrive drive) {
			this.drive = drive;
		}

		RobotDrive drive;

		@Override
		public void pidWrite(double output) {
			drive.arcadeDrive(0.0, output);
		}

	}

	PIDController angleController;

	public Drive() {
		rightFront = new CANTalon(RobotMap.DRIVE_RIGHT_FRONT);
		leftFront = new CANTalon(RobotMap.DRIVE_LEFT_FRONT);
		rightBack = new CANTalon(RobotMap.DRIVE_RIGHT_BACK);
		leftBack = new CANTalon(RobotMap.DRIVE_LEFT_BACK);
		
		driveInit();
		
		//teleopCommand = new DriveTeleop();
		
		//drive = new RobotDrive(leftFront, rightFront);
//		angleController = new PIDController(RobotMap.AIM_D, RobotMap.AIM_I, RobotMap.AIM_D,
//				new NavxUnwrappedAnglePIDSource(navx), new DrivePIDOutput(drive));

	}
	
	public void enableTeleop() {
		if (!teleopCommand.isRunning()) {
			teleopCommand.start();
		}
	}
	

	public boolean teleopIsRunning()
	{
		if (teleopCommand != null)
		{
			return teleopCommand.isRunning();
		}
		else
		{
			return false;
		}
	
	}
	
	
	public void disableTeleop()
	{
		if (teleopCommand.isRunning())
		{
			teleopCommand.cancel();
		}
	}

	
	public void driveInit() {
		
		initMotionMagic();
		
		rightFront.changeControlMode(TalonControlMode.PercentVbus);
		rightFront.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		rightFront.configEncoderCodesPerRev(250);
		//rightFront.setInverted(true);
		//leftFront.setInverted(true);

		rightBack.changeControlMode(TalonControlMode.Follower);
		rightBack.set(rightFront.getDeviceID());

		leftFront.changeControlMode(TalonControlMode.PercentVbus);
		leftFront.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		leftFront.configEncoderCodesPerRev(250);
		leftBack.changeControlMode(TalonControlMode.Follower);
		leftBack.set(leftFront.getDeviceID());

	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		// Drive system will allow joystick control when not stated otherwise.
		System.out.println("Init default command");
		teleopCommand = new DriveTeleop();
		setDefaultCommand(teleopCommand);
	}

	/**
	 * Set the drive mode of the front talons and set the back talons to
	 * follower mode
	 * 
	 * @param mode
	 *            The mode to set to
	 */
	public void setDriveMode(TalonControlMode mode) {
		rightFront.changeControlMode(mode);

		rightBack.changeControlMode(TalonControlMode.Follower);
		rightBack.set(rightFront.getDeviceID());

		leftFront.changeControlMode(mode);

		leftBack.changeControlMode(TalonControlMode.Follower);
		leftBack.set(leftFront.getDeviceID());

	}

	/**
	 * Drive using tank drive style controls
	 * 
	 * @param left
	 *            Left side mag
	 * @param right
	 *            Right side mag
	 */
	public void tankDrive(double left, double right) {
		if (enableTankDriveWithEncoders) {
			final int FULL_SPEED_ROTATION_INCREMENT = 3000;
			rightWheelRotations += left / FULL_SPEED_ROTATION_INCREMENT;
			leftWheelRotations += right / FULL_SPEED_ROTATION_INCREMENT;
			
			rightFront.set(rightWheelRotations);
			leftFront.set(leftWheelRotations);
		} else {
			rightFront.setMotionMagicCruiseVelocity(400);
			leftFront.setMotionMagicCruiseVelocity(400);
			
			rightFront.setMotionMagicAcceleration(400);
			leftFront.setMotionMagicAcceleration(400);
			
			rightFront.set(-right);
			leftFront.set(left);
		}
	}
	
	public void scootch(double throttle, double speed, double range) {
		double currentRange = throttle * range;
		if (currentRange > 0) {
			rightFront.set(speed);
			leftFront.set(speed);
		}
	}

	/**
	 * Drive using arcade drive style controls
	 * 
	 * @param move
	 *            Forward mag
	 * @param rotate
	 *            Rotate mag
	 */
//	public void arcadeDrive(double move, double rotate) {
//		drive.arcadeDrive(move, rotate);
//	}

	/**
	 * Enable control using the navx
	 */
	public void enableAnglePID() {
		angleController.enable();
	}

	/**
	 * Disable control using the navx
	 */
	public void disableAnglePID() {
		angleController.disable();
	}

	/**
	 * Set the setpoint of the navx
	 * 
	 * @param setPoint
	 */
	public void driveAnglePID(double setPoint) {
		angleController.setSetpoint(setPoint);
	}

	/**
	 * Check if the angle is within the setpoint by a tolerance.
	 * 
	 * @return true if within the setpoint false if not
	 */
	public boolean angleOnTarget() {
		System.out.println("Average Error " + Math.abs(angleController.getAvgError()));
		return Math.abs(angleController.getAvgError()) < ANGLE_ERROR_TOLERANCE;
		
	}

	/**
	 * Set pid values of navx angle control
	 * 
	 * @param p
	 * @param i
	 * @param d
	 */
	public void setAnglePID(double p, double i, double d) {
		angleController.setPID(p, i, d);
	}

	/**
	 * Stop driving and instruct the talons to run in motion magic mode
	 */
	public void initMotionMagic() {		
		//drive.tankDrive(0.0, 0.0);
		//drive.free();
		//drive.setExpiration(0);
		
		//drive.setLeftRightMotorOutputs(0, 0);
		//drive.stopMotor();
		//_talon.SetFeedbackDevice(CTRE.TalonSrx.FeedbackDevice.CtreMagEncoder_Relative);
		//SetSensorDirection ??

		//status |= _talon.SetIzone(kSlotIdx, 0, kTimeoutMs);
		
		
		
		//rightFront.changeControlMode(TalonControlMode.MotionMagic);
		//leftFront.changeControlMode(TalonControlMode.MotionMagic);
		
	
		setDriveMode(TalonControlMode.MotionMagic);
		
//	    rightFront.setMotionMagicCruiseVelocity(200.0);
//	    leftFront.setMotionMagicCruiseVelocity(200.0);
//	    rightFront.setMotionMagicAcceleration(400.0);
//	    leftFront.setMotionMagicAcceleration(400.0);
	    //rightFront.setInverted(true);
	    
	    //FOR COMPETITION BOT DO THE FOLLOWING
	    rightFront.reverseOutput(true);
	    leftFront.reverseOutput(true);
	    //END COMPETITION BOT
	     
	    //WE believe the following is the same as reverseOutput
	    //rightFront.reverseSensor(true);
	    //rightFront.reverseSensor(false);
	     
		//rightFront.reverseOutput(true);
		//leftFront.reverseOutput(true);
	     
		rightFront.setPosition(0.0);
		leftFront.setPosition(0.0);
	}

	/**
	 * Set pidf values of rightFront and leftFront talons. Puts robot in motion
	 * magic mode if not already in motion magic mode.
	 * 
	 * @param p
	 * @param i
	 * @param d
	 * @param f
	 */
	public void setMotionMagicPIDF(double p, double i, double d, double f) {
//		if (rightFront.getControlMode() != TalonControlMode.MotionMagic
//				&& leftFront.getControlMode() != TalonControlMode.MotionMagic) {
//			initMotionMagic();
//		}
		rightFront.setP(p);
		rightFront.setI(i);
		rightFront.setD(d);
		rightFront.setF(f);

		leftFront.setP(p);
		leftFront.setI(i);
		leftFront.setD(d);
		leftFront.setF(f);
	}

	/**
	 * Update the talon's motion magic parameters based on given parameters
	 * 
	 * @param rotationParams
	 */
	public void rotateMotionMagic(MotionParameters rotationParams) {

		rightFront.setMotionMagicCruiseVelocity(rotationParams.rightVelocity);
		leftFront.setMotionMagicCruiseVelocity(rotationParams.leftVelocity);

		rightFront.setMotionMagicAcceleration(rotationParams.rightAcceleration);
		leftFront.setMotionMagicAcceleration(rotationParams.leftAcceleration);

		rightFront.set(rotationParams.rightWheelRotations);
		leftFront.set(rotationParams.leftWheelRotations);
		
//		System.out.println("Right wheel rotations: " + rotationParams.rightWheelRotations +
//				"\tLeft wheel rotations: " + rotationParams.leftWheelRotations +
//				"\tRight acceleration: " + rotationParams.rightAcceleration +
//				"\tLeft acceleration: " + rotationParams.leftAcceleration +
//				"\tRight velocity: " + rotationParams.rightVelocity +
//				"\tOuter velocity: " + rotationParams.leftVelocity 
//				);

	}
	
	public void driveStraightMotionMagic(MotionParameters  driveParams) {
		rightFront.setMotionMagicCruiseVelocity(driveParams.rightVelocity);
		leftFront.setMotionMagicCruiseVelocity(driveParams.leftVelocity);
		
		rightFront.setMotionMagicAcceleration(driveParams.rightAcceleration);
		leftFront.setMotionMagicAcceleration(driveParams.leftAcceleration);
		
		rightFront.set(driveParams.rightWheelRotations);
		leftFront.set(driveParams.leftWheelRotations);
		
		//double talon1Error = Math.abs(driveParams.leftWheelRotations - rightFront.getPosition());
		//double talon2Error = Math.abs(driveParams.rightWheelRotations - leftFront.getPosition());
		
//		System.out.println("Right wheel rotations: " + driveParams.rightWheelRotations +
//				"\tLeft wheel rotations: " + driveParams.leftWheelRotations +
//				"\tRight acceleration: " + driveParams.rightAcceleration +
//				"\tLeft acceleration: " + driveParams.leftAcceleration +
//				"\tRight velocity: " + driveParams.rightVelocity +
//				"\tLeft velocity: " + driveParams.leftVelocity 
//				);

	}
	
	
	public void navxSetPoint(double heading) {
		
	}
	
	public boolean motionNavxFinished(double targetHeading) {
		double currentHeading = navx.getAngle();
		errNavxDrive = targetHeading - currentHeading;
		
		System.out.println("motionNavxFinished:targetHeading:" + targetHeading   + "\tcurrentHeading: " + currentHeading + "\terrNavxDrive: " + errNavxDrive);		
		
		return (Math.abs(errNavxDrive) < ANGLE_ERROR_TOLERANCE);
	}
	
	public void updateMotionNavx(double heading) {
		//this.navx.getHeading()
		//rightFront.set(rotationParams.innerWheelRotations);
		//leftFront.set(rotationParams.outerWheelRotations);
		double currentHeading = this.navx.getAngle();
		
		errNavxDrive = heading - currentHeading;
		
		//if (Math.abs(errNavxDrive) < ANGLE_ERROR_TOLERANCE)
		//{
			double p = 0.01;
			double output = -errNavxDrive * p;
			System.out.println("updateMotionNavx:output:" + output );

			//drive.arcadeDrive(0.0, output);
		//}
		

		
		
		
		//angleController.setSetpoint(heading);
		//angleController.enable();
		 
		//driveAnglePID(heading)
		
	}
	



	/**
	 * Checks if motion magic routine is finished based on given parameters.
	 * 
	 * @param rotationParams
	 * @param errorTolerance
	 *
	 * @return true if  false if not.
	 */
	public boolean motionMagicDone(MotionParameters rotationParams, double errorTolerance) {

		double rightFrontPosition = rightFront.getPosition();
		double leftFrontPosition = leftFront.getPosition();
		
		//double encRightFrontPosition = rightFront.getEncPosition()*10;
		//double encLeftFrontPosition = leftFront.getEncPosition()*10;
		
		//double rightDelta = rightFrontPosition - encRightFrontPosition;
		//double leftDelta =  leftFrontPosition - encLeftFrontPosition;
		//double rightError = rightFront.getError();
		//double leftError = leftFront.getError();
		
		
		double rightFrontError = Math.abs(rotationParams.rightWheelRotations - rightFrontPosition);
		double leftFrontError = Math.abs(rotationParams.leftWheelRotations - leftFrontPosition);
		
		
		//System.out.println("rightError:" + rightError + "\tleftError:" + leftError + "\trightFrontError:" + rightFrontError + "\t leftFrontError:" + leftFrontError + "\t rightFrontPosition:" + rightFrontPosition + "\t leftFrontPosition:" + leftFrontPosition);
		
		return (rightFrontError < errorTolerance && leftFrontError < errorTolerance);
		//return (leftFrontError < MOTION_MAGIC_ERROR_TOLERANCE);
		//talon1Error = Math.abs(rotationParams.innerWheelRotations - _talon.getPosition());
		//talon2Error = Math.abs(rotationParams.outerWheelRotations - _talon2.getPosition());
		
	}
	
//	public boolean motionNavxRoutineDone(RotationParameters rotationParams){
//		
//	}
	
	public Navx getNavx() {
		return navx;
	}
	

	/**
	 * Modify the talon based on the consumer
	 * 
	 * @param modifier
	 *            A consumer that takes the given talon and performs a
	 *            modification on its parameters.
	 */
	public void modRightFrontTalon(Consumer<CANTalon> modifier) {
		modifier.accept(rightFront);
	}

	/**
	 * Modify the talon based on the consumer
	 * 
	 * @param modifier
	 *            A consumer that takes the given talon and performs a
	 *            modification on its parameters.
	 */
	public void modLeftFrontTalon(Consumer<CANTalon> modifier) {
		modifier.accept(leftFront);
	}

	/**
	 * Modify the talon based on the consumer
	 * 
	 * @param modifier
	 *            A consumer that takes the given talon and performs a
	 *            modification on its parameters.
	 */
	public void modRightBackTalon(Consumer<CANTalon> modifier) {
		modifier.accept(rightBack);
	}

	/**
	 * Modify the talon based on the consumer
	 * 
	 * @param modifier
	 *            A consumer that takes the given talon and performs a
	 *            modification on its parameters.
	 */
	public void modLeftRearTalon(Consumer<CANTalon> modifier) {
		modifier.accept(leftBack);
	}

	/**
	 * Get a value based on the given talon
	 * 
	 * @param getter
	 *            A function that performs an operation on the talon that gets
	 *            its information and returns it
	 * @return The value gotten from the getter
	 */
	public <Type> Type getRightFrontTalon(Function<CANTalon, Type> getter) {
		return getter.apply(rightFront);
	}

	/**
	 * Get a value based on the given talon
	 * 
	 * @param getter
	 *            A function that performs an operation on the talon that gets
	 *            its information and returns it
	 * @return The value gotten from the getter
	 */
	public <Type> Type getRightBackTalon(Function<CANTalon, Type> getter) {
		return getter.apply(rightBack);
	}

	/**
	 * Get a value based on the given talon
	 * 
	 * @param getter
	 *            A function that performs an operation on the talon that gets
	 *            its information and returns it
	 * @return The value gotten from the getter
	 */
	public <Type> Type getLeftFrontTalon(Function<CANTalon, Type> getter) {
		return getter.apply(leftFront);
	}

	/**
	 * Get a value based on the given talon
	 * 
	 * @param getter
	 *            A function that performs an operation on the talon that gets
	 *            its information and returns it
	 * @return The value gotten from the getter
	 */
	public <Type> Type getleftBackTalon(Function<CANTalon, Type> getter) {
		return getter.apply(leftBack);
	}

}
