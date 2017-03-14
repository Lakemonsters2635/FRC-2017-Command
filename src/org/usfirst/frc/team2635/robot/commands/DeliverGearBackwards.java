package org.usfirst.frc.team2635.robot.commands;

import org.usfirst.frc.team2635.robot.Robot;

import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 * Set the gear deliverer in the backwards position
 */
public class DeliverGearBackwards extends TimedCommand {

    public DeliverGearBackwards(double timeout) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	super(timeout);
    	requires(Robot.deliverer);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.deliverer.setBackwards();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	boolean isTimedOut = this.isTimedOut();
    	return isTimedOut;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
