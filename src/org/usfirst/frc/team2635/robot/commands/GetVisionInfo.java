package org.usfirst.frc.team2635.robot.commands;

import org.usfirst.frc.team2635.robot.Robot;

import org.usfirst.frc.team2635.robot.model.VisionParameters;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class GetVisionInfo extends Command {


	public VisionParameters visionParameters; 
	public String targetName;
	public double duration;
	Timer timer;
	Double averageAquiredAngle;
	int sampleCount;
	Double currentAngleSample;
	
    public GetVisionInfo(VisionParameters visionParams, String targetName, double duration) {
        // Use requires() here to declare subsystem dependencies
    	//requires(Robot.vision);
    	this.visionParameters = visionParams;
    	this.targetName = targetName;
    	this.duration = duration;
    	timer = new Timer();
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	timer.reset();
    	timer.start();
    	sampleCount = 0;
    	averageAquiredAngle = null;
    	
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	
    	
     	Robot.light.lightOn();
 
    	if (targetName == "Gear")
    	{    	
    		Robot.vision.gearAim();
    		
    		// 
    		
    		currentAngleSample = Robot.vision.getAngleToGear();
    		visionParameters.DistanceToTarget = Robot.vision.getDistanceToGear();
    	}
    	else if (targetName == "Boiler")
    	{
    		Robot.vision.aim();
    		currentAngleSample = Robot.vision.getAngleToBoiler();
    		visionParameters.DistanceToTarget = Robot.vision.getDistanceToBoiler();
    	}
    	
    	
    	 System.out.println("currentAngleSample: " + currentAngleSample);
    	if (currentAngleSample != null)
    	{
    		sampleCount++;
    		if (sampleCount == 1)
    		{
    			averageAquiredAngle = 0.0;
    		}
    		averageAquiredAngle =  (averageAquiredAngle * (sampleCount-1) +  currentAngleSample)/sampleCount;
    	}
    	
    	//visionParameters.AngleToTarget averageAquiredAngle 
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	
    	
    	 //boolean timeElapsed =  (timer.get() * 1000 > duration);
    	 boolean timeElapsed = timer.hasPeriodPassed(duration);
    	 boolean isDone = false;
    	 if (timeElapsed)
    	 {
    		 isDone = true;
    		 if (averageAquiredAngle == null)
    		 {
    			 averageAquiredAngle = 0.0;
    			 System.out.println("WARNING:averageAquiredAngle is NULL. Setting to 0.0");
    		 }
    		 visionParameters.AngleToTarget = averageAquiredAngle;
    		 System.out.println("visionParameters.AngleToTarget: " + visionParameters.AngleToTarget);
    	 }
    	
    	
    	 return isDone;

         
    }

    // Called once after isFinished returns true
    protected void end() {
    	timer.stop();
    	
    	Robot.light.lightOff();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.light.lightOff();
    	timer.stop();
    	timer.reset();
    }
}
