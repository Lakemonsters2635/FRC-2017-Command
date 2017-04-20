package org.usfirst.frc.team2635.robot.commands;

import org.usfirst.frc.team2635.robot.Robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Set the fire piston forward
 */
public class ShooterFire extends Command {
	Value value;
	
    public ShooterFire(Value value) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.shooter);
    	this.value = value;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.shooter.setFireControl(value);
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
