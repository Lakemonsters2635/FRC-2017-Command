package org.usfirst.frc.team2635.robot.commands;


import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2635.robot.Robot;

/**
 * Set the ball pickup to a certain magnitude
 */
public class PickupBall extends Command {

    double magnitude;

    public PickupBall(double magnitude) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.pickup);
        this.magnitude = magnitude;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Robot.pickup.feed(magnitude);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.pickup.feed(0.0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        Robot.pickup.feed(0.0);
    }
}
