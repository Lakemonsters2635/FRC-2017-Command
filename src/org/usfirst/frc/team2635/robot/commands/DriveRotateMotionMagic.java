package org.usfirst.frc.team2635.robot.commands;

import java.util.function.Consumer;
import java.util.function.Function;

import org.usfirst.frc.team2635.robot.Robot;
import org.usfirst.frc.team2635.robot.RobotMap;
import org.usfirst.frc.team2635.robot.model.MotionProfileLibrary;
import org.usfirst.frc.team2635.robot.model.VisionLight;
import org.usfirst.frc.team2635.robot.model.VisionParameters;
import org.usfirst.frc.team2635.robot.model.MotionParameters;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Rotate using motion magic
 */
public class DriveRotateMotionMagic extends Command {
	double rpm;
	double targetAngle;
	VisionParameters visionParams;
	double turnRadiusInches;
	boolean clockwise;
	boolean rotateCenter;
	public boolean hasExecuted;
	

	
	MotionParameters rotationParams; 
	
    public DriveRotateMotionMagic(double rpm, double targetAngle, double turnRadiusInches, boolean clockwise, boolean rotateCenter) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.drive);

    	
    	this.rpm = rpm;
    	this.targetAngle = targetAngle;
    	this.turnRadiusInches = turnRadiusInches;
    	this.clockwise = clockwise;
    	this.rotateCenter = rotateCenter; 
    }
    
    public DriveRotateMotionMagic(double rpm, VisionParameters visionParams) {
    	this.visionParams = visionParams;
    	this.rpm = rpm;
    	this.targetAngle = 0;
    	this.turnRadiusInches = 0;
    	this.clockwise = true;
    	this.rotateCenter = true; 
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("DriveRotateMotionMagic initialize");
    	




    	if (targetAngle == 0 && visionParams != null && visionParams.AngleToTarget != null) {
    		targetAngle = visionParams.AngleToTarget;
    		System.out.println("DriveRotate.visionParams.AngleToTarget:" + visionParams.AngleToTarget);
    	}
    	
    		
    		
    	
    	
    	System.out.println("DriveRotate.targetAngle:" + targetAngle);
    	rotationParams = MotionProfileLibrary.getRotationParameters(targetAngle,
				RobotMap.WHEEL_RADIUS_INCHES, turnRadiusInches, RobotMap.WHEEL_SEPARATION_INCHES, rpm, clockwise,
				rotateCenter);
    	
    	Robot.drive.DriveInit();
    	Robot.drive.initMotionMagic();
    	Robot.drive.setMotionMagicPIDF(
    			RobotMap.MOTION_MAGIC_P,
    			RobotMap.MOTION_MAGIC_I,
    			RobotMap.MOTION_MAGIC_D,
    			RobotMap.MOTION_MAGIC_F);
    	Robot.drive.rotateMotionMagic(rotationParams);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//System.out.println("DriveRotateMotionMagic execute");
    	Robot.drive.rotateMotionMagic(rotationParams);   	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	boolean done = Robot.drive.motionMagicDone(rotationParams, Robot.drive.ROTATE_ERROR_TOLERANCE);
    	if (done) {
    		System.out.println("DriveRotateMotionMagic is done");
        	rpm = 0;
        	targetAngle = 0;
        	turnRadiusInches = 0;
        	if (visionParams != null)
        	{
	    		visionParams.AngleToTarget = null;
	    		visionParams.DistanceToTarget = null;
        	}
    	}
    	return done;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("DriveRotateMotionMagic end");
    	Robot.drive.setDriveMode(TalonControlMode.PercentVbus);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	System.out.println("DriveRotateMotionMagic interrupted");
    	rpm = 0;
    	targetAngle = 0;
    	turnRadiusInches = 0;
    	Robot.drive.setDriveMode(TalonControlMode.PercentVbus);
    }
}