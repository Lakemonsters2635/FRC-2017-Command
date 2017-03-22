package org.usfirst.frc.team2635.robot.commands;

import com.ctre.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2635.robot.Robot;
import org.usfirst.frc.team2635.robot.RobotMap;
import org.usfirst.frc.team2635.robot.model.MotionParameters;
import org.usfirst.frc.team2635.robot.model.MotionProfileLibrary;
import org.usfirst.frc.team2635.robot.model.SensorParameters;
import org.usfirst.frc.team2635.robot.subsystems.Drive;

import java.time.LocalDateTime;

/**
 *
 */
public class DriveStraightMotionMagic extends Command {
	public double driveDistance;
	public boolean reverse;
	double rpm;
	public int cycleCtr;
	SensorParameters sensorParams;
	
	MotionParameters driveParams; 
	
    public DriveStraightMotionMagic(double rpm, double driveDistance, boolean reverse) {
        requires(Robot.drive);
    	this.rpm = rpm;
    	this.driveDistance = driveDistance;
    	this.reverse = reverse;
    }

    public DriveStraightMotionMagic(double rpm, SensorParameters sensorParams) {
    	this.sensorParams = sensorParams;
    	this.rpm = rpm;
    	this.driveDistance = 0;
    	this.reverse = false;
    	
    }
   
    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("DrivesStraightMotionMagic initialized at "  + LocalDateTime.now());
       	

    	
    	if (sensorParams != null && sensorParams.DistanceToTarget != null)
    	{
    		
    		//targetAngle = visionParams.AngleToTarget;
    		
    		System.out.println("Get Drive Params by SensorParameters:" + sensorParams.DistanceToTarget + sensorParams.DistanceAdjustment);
    		
    		driveParams = MotionProfileLibrary.getDriveParameters(RobotMap.WHEEL_RADIUS_INCHES, sensorParams.DistanceToTarget + sensorParams.DistanceAdjustment, rpm, reverse);
   
    	}
    	else
    	{
       		System.out.println("Get Drive Params by fixed distance:" + driveDistance);
    		driveParams = MotionProfileLibrary.getDriveParameters(RobotMap.WHEEL_RADIUS_INCHES, driveDistance, rpm, reverse);
    	}
    	
    	

    	Robot.drive.initMotionMagic();
    	Robot.drive.setMotionMagicPIDF(
    			RobotMap.DRIVE_STRAIGHT_MOTION_MAGIC_P,
    			RobotMap.DRIVE_STRAIGHT_MOTION_MAGIC_I,
    			RobotMap.DRIVE_STRAIGHT_MOTION_MAGIC_D,
    			RobotMap.DRIVE_STRAIGHT_MOTION_MAGIC_F);
    	
    	Robot.drive.driveStraightMotionMagic(driveParams);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

    	

    	
    }

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		boolean done = Robot.drive.motionMagicDone(driveParams, Drive.DRIVE_ERROR_TOLERANCE);
		if (done) {
    		System.out.println("DriveStraightMotionMagic is done");

    		if (sensorParams != null)
    		{
    			sensorParams.DistanceToTarget = null;
	    		//sensorParams.AngleToTarget = null;
	    		//DONT NULL OUT ANGLE IN DRIVE IN ROTATE COMMAND
    		}
    		


    	}
    	return done;
    	//return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("DriveStraightMotionMagic end");
    	Robot.drive.initMotionMagic();
    	Robot.drive.setDriveMode(TalonControlMode.PercentVbus);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	//System.out.println("DriveStraightMotionMagic interrupted");
    	Robot.drive.setDriveMode(TalonControlMode.PercentVbus);
    }
}
