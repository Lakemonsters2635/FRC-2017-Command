package org.usfirst.frc.team2635.robot.commands;

import org.usfirst.frc.team2635.robot.Robot;
import org.usfirst.frc.team2635.robot.model.VisionLight;
import org.usfirst.frc.team2635.robot.model.VisionParameters;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class GetVisionInfo extends Command {

	//FHE TODO: Implement timeout.
	public VisionParameters visionParameters; 
	public String targetName;
	VisionLight light;
	
    public GetVisionInfo(VisionParameters visionParams, String targetName) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.visionParameters = visionParams;
    	this.targetName = targetName;
    	light = new VisionLight(7);
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	
    	
     	light.lightOn();
 
    	if (targetName == "Gear")
    	{    	
    		Robot.vision.gearAim();
    		visionParameters.AngleToTarget = Robot.vision.getAngleToGear();
    		visionParameters.DistanceToTarget = Robot.vision.getDistanceToGear();
    	}
    	else if (targetName == "Boiler")
    	{
    		Robot.vision.aim();
    		visionParameters.AngleToTarget = Robot.vision.getAngleToBoiler();
    		visionParameters.DistanceToTarget = Robot.vision.getDistanceToBoiler();
    	}
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	 boolean isDone = visionParameters.AngleToTarget != null;
    	 if (isDone)
    	 {
    		 System.out.println("visionParameters.AngleToTarget: " + visionParameters.AngleToTarget);
    	 }
         return isDone;
    }

    // Called once after isFinished returns true
    protected void end() {
    	light.lightOff();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	light.lightOff();
    }
}
