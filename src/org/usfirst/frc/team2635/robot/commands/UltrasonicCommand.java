package org.usfirst.frc.team2635.robot.commands;

import org.usfirst.frc.team2635.robot.Robot;
import org.usfirst.frc.team2635.robot.model.SensorParameters;
import org.usfirst.frc.team2635.robot.model.UltrasonicParameters;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 *
 */
public class UltrasonicCommand extends TimedCommand {
	public SensorParameters sensorParameters;
	
    public UltrasonicCommand(SensorParameters params, double timeout) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	super(timeout);
    	this.sensorParameters = params;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if (sensorParameters != null)
    	{
    		sensorParameters.DistanceToTarget = null;
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//ultrasonicParameters.leftInches =  Robot.ultrasonic.getLeftDistanceInches();
    	sensorParameters.DistanceToTarget = Robot.ultrasonic.getRightDistanceInches();

    	

		if (sensorParameters.DistanceToTarget == null)
		{
			sensorParameters.DistanceToTarget = new Double(0.0);	
		}
		else if (sensorParameters.DistanceToTarget <= 11)
		{
			sensorParameters.DistanceToTarget = 0.0;
		}
    	
  
    }






    // Called once after timeout
    protected void end() {
    	
		System.out.println("Ultrasonic:DistanceToTarget:" + sensorParameters.DistanceToTarget);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
