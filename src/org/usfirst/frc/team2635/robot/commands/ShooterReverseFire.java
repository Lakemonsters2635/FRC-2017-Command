package org.usfirst.frc.team2635.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2635.robot.Robot;

/**
 * Set the fire piston backwards
 */
public class ShooterReverseFire extends Command {

    public ShooterReverseFire() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.shooter);
    }

    // Called just before this Command runs the first time
    protected void initialize() {


    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Robot.shooter.fireControlReverse();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        //Set and done baby
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
