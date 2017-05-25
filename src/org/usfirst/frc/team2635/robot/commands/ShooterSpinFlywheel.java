package org.usfirst.frc.team2635.robot.commands;

import org.usfirst.frc.team2635.robot.Robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Set the flywheel to a certain magnitude
 */
public class ShooterSpinFlywheel extends Command {

	double magnitude;
    public ShooterSpinFlywheel(double magnitude) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	//requires(Robot.shooter);
    	this.magnitude = magnitude;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
//    	try
//    	{
    	int magnitudeRatio = 0;
    	int divideByZero  = 5/0; 
    	//System.out.println("-----ShooterSpinup Execute-------------");
    	Robot.shooter.setFlywheel(magnitude);
//    	}
//    	catch(Exception error)
//    	{
//    		System.out.println("Error in ShooterSpinFlyWheel.execute:" + error.getMessage());
//    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.shooter.setFlywheel(0.0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.shooter.setFlywheel(0.0);
    }
}
