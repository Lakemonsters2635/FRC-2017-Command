package org.usfirst.frc.team2635.robot.commands;

import org.usfirst.frc.team2635.robot.Robot;
import org.usfirst.frc.team2635.robot.model.UltrasonicParameters;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class UltrasonicCommand extends Command {
	public UltrasonicParameters ultrasonicParameters;
	
    public UltrasonicCommand(UltrasonicParameters params) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.ultrasonicParameters = params;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	ultrasonicParameters.leftInches =  Robot.ultrasonic.getLeftDistanceInches();
    	ultrasonicParameters.rightInches = Robot.ultrasonic.getRightDistanceInches();
    	
    	System.out.println("leftInches:" + ultrasonicParameters.leftInches + "\trightInches:" + ultrasonicParameters.rightInches);
    	
  
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
