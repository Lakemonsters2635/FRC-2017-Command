package org.usfirst.frc.team2635.robot.commands;

import org.usfirst.frc.team2635.robot.OI;
import org.usfirst.frc.team2635.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Climb based on a certain magnitude
 */
public class ClimberClimb extends Command {
	double magnitude;
    public ClimberClimb(double magnitude) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.climber);
    	this.magnitude = magnitude;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        // Map range (+1:-1) to (0:-1) so that full range of axis is 0 to -1
    	double climbspeed = (Robot.oi.getLeftZ()-1.0)/2.0;
    	Robot.climber.climb(climbspeed);
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.climber.climb(0.0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.climber.climb(0.0);
    }
}
