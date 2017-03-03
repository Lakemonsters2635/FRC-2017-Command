package org.usfirst.frc.team2635.robot.commands;

import org.usfirst.frc.team2635.robot.Robot;
import org.usfirst.frc.team2635.robot.RobotMap;
import org.usfirst.frc.team2635.robot.model.DriveParameters;
import org.usfirst.frc.team2635.robot.model.MotionParameters;
import org.usfirst.frc.team2635.robot.model.MotionProfileLibrary;


import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveStraightMotionMagic extends Command {

	
	public double driveDistance;
	public boolean reverse;
	double rpm;

	
	MotionParameters driveParams; 
	
	
	
	
    public DriveStraightMotionMagic(double rpm, double driveDistance, boolean reverse) {
    	
    	this.rpm = rpm;
    	this.driveDistance = driveDistance;
    	this.reverse = reverse;
    	
    	driveParams = MotionProfileLibrary.getDriveParameters(RobotMap.WHEEL_RADIUS_INCHES, driveDistance, rpm, reverse);

    }


    	



    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("DrivesStraightMotionMagic initialize");
    	Robot.drive.initMotionMagic();
    	Robot.drive.setMotionMagicPIDF(
    			RobotMap.MOTION_MAGIC_P,
    			RobotMap.MOTION_MAGIC_I,
    			RobotMap.MOTION_MAGIC_D,
    			RobotMap.MOTION_MAGIC_F);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	System.out.println("DriveStraightMotionMagic execute");
    	Robot.drive.driveStraightMotionMagic(driveParams);
    
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	boolean done = Robot.drive.motionMagicDone(driveParams);
    	System.out.print("DriveStraightMotionMagic is " + (done?"done":"not done"));

    	return done;
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
    	System.out.println("DriveStraightMotionMagic interrupted");
    	Robot.drive.setDriveMode(TalonControlMode.PercentVbus);
    }
}
