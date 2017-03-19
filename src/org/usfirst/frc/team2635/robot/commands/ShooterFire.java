package org.usfirst.frc.team2635.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2635.robot.Robot;

/**
 * Set the fire piston forward
 */
public class ShooterFire extends Command {

    public ShooterFire() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.shooter);
    }

    // Called just before this Command runs the first time
    protected void initialize() {

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Robot.shooter.fireControlForward();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        //Set and done baby
        System.out.println("Shooter Fire isFinished");
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
        System.out.println("Shooter Fire end");
        //Robot.shooter.fireControlForward();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
