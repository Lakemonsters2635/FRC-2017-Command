package org.usfirst.frc.team2635.robot.commands;

import org.usfirst.frc.team2635.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ManageTeleop extends Command {

	public boolean setTeleopEnabled;
    public ManageTeleop(boolean setTeleopEnabled) {
    	requires(Robot.drive);
    	this.setTeleopEnabled = setTeleopEnabled;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if (setTeleopEnabled)
    	{
    		Robot.drive.enableTeleop();
    	}
    	else
    	{
    		Robot.drive.disableTeleop();
    	}
    		
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
