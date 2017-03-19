package org.usfirst.frc.team2635.robot.commands;

import org.usfirst.frc.team2635.robot.Robot;

import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 * Set the gear deliverer in the backwards position
 */
public class DeliverGearBackwards extends TimedCommand {

    public DeliverGearBackwards(double timeout) {
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

    // Called once after timeout
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}