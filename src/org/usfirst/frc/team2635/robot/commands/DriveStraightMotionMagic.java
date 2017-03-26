package org.usfirst.frc.team2635.robot.commands;

import java.time.LocalDateTime;

import org.usfirst.frc.team2635.robot.Robot;
import org.usfirst.frc.team2635.robot.RobotMap;
import org.usfirst.frc.team2635.robot.model.DriveParameters;
import org.usfirst.frc.team2635.robot.model.MotionParameters;
import org.usfirst.frc.team2635.robot.model.MotionProfileLibrary;
import org.usfirst.frc.team2635.robot.model.SensorParameters;
import com.ctre.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.command.Command;

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
    		
    		double computedDriveDistance = sensorParams.DistanceToTarget;
    		if (!sensorParams.TargetAcquired && sensorParams.SensorFailedDriveDistance != null)
    		{
    			computedDriveDistance = sensorParams.SensorFailedDriveDistance;
    			System.out.println("Sensor Failed. Reverting to SensorFailedDriveDistance:" + sensorParams.SensorFailedDriveDistance);
    		}
    		else if (sensorParams.TargetAcquired)
    		{
    			//Only apply the distance Adjustment if we obtained a drive Distance from Vision.
    			computedDriveDistance = sensorParams.DistanceToTarget +  sensorParams.DistanceAdjustment;
    			if (computedDriveDistance < 0)
    			{
    				//If the adjusted distance < 0, something has gone wrong.
    				System.out.println("Vision says it succeeded, but the value is too low. Reverting to SensorFailedDriveDistance:" + sensorParams.SensorFailedDriveDistance);
    				computedDriveDistance = sensorParams.SensorFailedDriveDistance;
    			}
    		}
    		
    		driveParams = MotionProfileLibrary.getDriveParameters(RobotMap.WHEEL_RADIUS_INCHES, computedDriveDistance, rpm, reverse);
   
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
    	

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

    	
    	Robot.drive.driveStraightMotionMagic(driveParams);
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	boolean done = Robot.drive.motionMagicDone(driveParams, Robot.drive.DRIVE_ERROR_TOLERANCE);
    	if (done) {
    		if (RobotMap.DEBUG_DETAIL)
    		{
    			System.out.println("DriveStraightMotionMagic is done");
    		}

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
		if (RobotMap.DEBUG_DETAIL)
		{
			System.out.println("DriveStraightMotionMagic end");
		}
    	Robot.drive.initMotionMagic();
    	Robot.drive.setDriveMode(TalonControlMode.PercentVbus);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	System.out.println("DriveStraightMotionMagic interrupted");
    	Robot.drive.setDriveMode(TalonControlMode.PercentVbus);
    }
}
