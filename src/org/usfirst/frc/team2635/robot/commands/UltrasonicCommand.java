package org.usfirst.frc.team2635.robot.commands;

import org.usfirst.frc.team2635.robot.Robot;
import org.usfirst.frc.team2635.robot.model.UltrasonicParameters;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 *
 */
public class UltrasonicCommand extends TimedCommand {
	public UltrasonicParameters ultrasonicParameters;
	
    public UltrasonicCommand(UltrasonicParameters params, double timeout) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	super(timeout);
    	this.ultrasonicParameters = params;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//ultrasonicParameters.leftInches =  Robot.ultrasonic.getLeftDistanceInches();
    	ultrasonicParameters.rightInches = Robot.ultrasonic.getRightDistanceInches();

    	

		if (ultrasonicParameters.rightInches == null)
		{
			ultrasonicParameters.rightInches = new Double(0.0);	
		}
		else if (ultrasonicParameters.rightInches <= 11)
		{
			ultrasonicParameters.rightInches = 0.0;
		}
    	
  
    }






    // Called once after timeout
    protected void end() {
    	
		System.out.println("rightInches:" + ultrasonicParameters.rightInches);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
