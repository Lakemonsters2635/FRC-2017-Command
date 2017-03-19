package org.usfirst.frc.team2635.robot.commands;

import java.time.LocalDateTime;
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
	public boolean hasExecuted;


	
	MotionParameters rotationParams; 
	

    public DriveRotateMotionMagic(double rpm, double targetAngle) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.drive);
    	this.rpm = rpm;
    	this.targetAngle = targetAngle;
   }
    
    public DriveRotateMotionMagic(double rpm, VisionParameters visionParams) {
    	this.visionParams = visionParams;
    	this.rpm = rpm;
    	this.targetAngle = 0;

    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    	System.out.println("DriveRotateMotionMagic initialize started at " + LocalDateTime.now());
    	hasExecuted = false;
    	if (visionParams != null && visionParams.AngleToTarget != null)
    	{
    		
    		//targetAngle = visionParams.AngleToTarget;
    		System.out.println("Get Rotation by VisionParams.AngleToTarget:" + visionParams.AngleToTarget);
    	   	rotationParams = MotionProfileLibrary.getRotationParameters(visionParams.AngleToTarget,
    				RobotMap.WHEEL_RADIUS_INCHES, RobotMap.WHEEL_SEPARATION_INCHES, rpm);

    	}
    	else
    	{
    		System.out.println("Get Rotation by fixed:" + targetAngle);
    		rotationParams = MotionProfileLibrary.getRotationParameters(targetAngle,
    				RobotMap.WHEEL_RADIUS_INCHES, RobotMap.WHEEL_SEPARATION_INCHES, rpm);
    	}
    	
    	Robot.drive.initMotionMagic();
    	Robot.drive.setMotionMagicPIDF(
    			RobotMap.MOTION_MAGIC_P,
    			RobotMap.MOTION_MAGIC_I,
    			RobotMap.MOTION_MAGIC_D,
    			RobotMap.MOTION_MAGIC_F);
    	
    	Robot.drive.rotateMotionMagic(rotationParams);   
    	System.out.println("DriveRotateMotionMagic initialize ended at " + LocalDateTime.now());
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (hasExecuted)
    	{
    		System.out.println("DriveRotateMotionMagic execution started at " + LocalDateTime.now());
    		hasExecuted = true;
    	}
    	//Robot.drive.rotateMotionMagic(rotationParams);   
       
        	


    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	boolean done = Robot.drive.motionMagicDone(rotationParams, Robot.drive.ROTATE_ERROR_TOLERANCE);
    	if (done) {
    		System.out.println("DriveRotateMotionMagic is done at " + LocalDateTime.now());
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
    	System.out.println("DriveRotateMotionMagic ended at " + LocalDateTime.now());
    	Robot.drive.initMotionMagic();
    	Robot.drive.setDriveMode(TalonControlMode.PercentVbus);
  
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	System.out.println("DriveRotateMotionMagic interrupted at " + LocalDateTime.now());
    	if (visionParams != null)
    	{
    		visionParams.AngleToTarget = null;
    		visionParams.DistanceToTarget = null;
    	}
    	Robot.drive.setDriveMode(TalonControlMode.PercentVbus);
    }
}