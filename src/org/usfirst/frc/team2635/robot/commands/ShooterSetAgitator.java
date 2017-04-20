package org.usfirst.frc.team2635.robot.commands;

import org.usfirst.frc.team2635.robot.Robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ShooterSetAgitator extends Command {

	Value value;
    public ShooterSetAgitator(Value value) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.shooter);
        this.value = value;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.shooter.setAgitator(value);
    	System.out.println("agitator init. value is " + value);

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	System.out.println("agitator exe.");
    
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