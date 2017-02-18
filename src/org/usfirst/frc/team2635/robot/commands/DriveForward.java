package org.usfirst.frc.team2635.robot.commands;


import org.usfirst.frc.team2635.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveForward extends Command {

	double rpm;
	boolean reverse;
	double driveDistanceInches;
	
	Timer timer;
    public DriveForward(double time, double mag) {
    	requires(Robot.drive);
    	timer = new Timer();
    	this.driveTime = time;
    	this.mag = mag;
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	timer.reset();
    	timer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.drive.tankDrive(mag, mag);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return timer.hasPeriodPassed(driveTime);
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drive.tankDrive(0, 0);
    	timer.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.drive.tankDrive(0, 0);
    	timer.stop();
    	timer.reset();
    }
}
