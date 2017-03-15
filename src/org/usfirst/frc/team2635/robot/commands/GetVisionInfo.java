package org.usfirst.frc.team2635.robot.commands;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.TreeSet;

import org.usfirst.frc.team2635.robot.Robot;

import org.usfirst.frc.team2635.robot.model.VisionParameters;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class GetVisionInfo extends Command {


	ArrayList<Double>  angleSamples;
	ArrayList<Double>  distanceSamples;
	public VisionParameters visionParameters; 
	public String targetName;
	public double duration;
	Timer timer;
	//Double averageAquiredAngle;
	Double averageAquiredDistance;
	
	int sampleCount;

	
	boolean hasExecuted;
	
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
    	hasExecuted = false;
    	timer.reset();
    	timer.start();
    	sampleCount = 0;
    	angleSamples = new ArrayList<Double>();
    	distanceSamples = new ArrayList<Double>();
     	Robot.light.lightOn();

    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Double angle = null;
    	Double distance = null;
    	Double roundedAngle = null;
    	Double roundedDistance = null;
    	if (targetName == "Gear")
    	{    	
    		Robot.vision.gearAim();
    		angle = Robot.vision.getAngleToGear();
    		distance = Robot.vision.getDistanceToGear();
    		if (angle!= null){
    		     roundedAngle = roundit(angle, 2);
    		     //System.out.println("roundedAngle: " + roundedAngle);
   				angleSamples.add(angle);
    		}
    		else
    		{
    			System.out.println("angle is NULL");
    		}
    		
    		
    		if (distance!= null){
    			distanceSamples.add(distance);
    		}
    		
   		
    	}
    	else if (targetName == "Boiler")
    	{
    		Robot.vision.aim();
    		angle = Robot.vision.getAngleToBoiler();
    		distance = Robot.vision.getDistanceToBoiler();
    		
    		if (angle!= null){
    			
    			angleSamples.add(angle);
    		}
    		if (distance!= null){
    			distanceSamples.add(distance);
    		}
    	}
    	
    	
	   	 //
	 
	   	
	   	//System.out.println("roundedAngle: " + roundedAngle + "\t roundedDistance:" + roundit(distance, 2));
    	//visionParameters.AngleToTarget averageAquiredAngle 
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	
    
    	
    	//System.out.println("currentAngleSample: " + currentAngleSample + "\t currentDistance:" + visionParameters.DistanceToTarget);

    	
    
    	 boolean timeElapsed = timer.hasPeriodPassed(duration);
    	 boolean isDone = false;
    	 if (timeElapsed)
    	 {
    		 isDone = true;
    		 Double modeAngle = modeit(angleSamples);
    		 System.out.println("modeAngle: " + modeAngle);
    		 
    		 visionParameters.AngleToTarget = modeit(angleSamples);
    		 visionParameters.DistanceToTarget = modeit(distanceSamples);
    		 
    		 System.out.println("visionParameters.AngleToTarget: " + visionParameters.AngleToTarget + "\t visionParameters.DistanceToTarget:" + visionParameters.DistanceToTarget);
    	 }
    	
    	
    	 return isDone;

         
    }

    // Called once after isFinished returns true
    protected void end() {
    	timer.stop();
    	angleSamples.clear();
    	distanceSamples.clear();
    	Robot.light.lightOff();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.light.lightOff();
    	timer.stop();
    	timer.reset();
    	
    	angleSamples.clear();
    	distanceSamples.clear();
    }
    
    double roundit(double num, double N)
    {
        double d = Math.log10(num);
        double power;
        if (num > 0)
        {
            d = Math.ceil(d);
            power = -(d-N);
        }
        else
        {
            d = Math.floor(d); 
            power = -(d-N);
        }

        return (double)(num * Math.pow(10.0, power) + 0.5) * Math.pow(10.0, -power);
    }
    
    Double modeit(ArrayList<Double> samples){
    	
    	 
        // list of all the numbers 
        List<Double> list = new ArrayList<Double>();
   
        // list of all the numbers with no duplicates
        TreeSet<Double> tree = new TreeSet<Double>();
   
        for (int i = 0; i < samples.size(); i++) {
           list.add(samples.get(i));
           tree.add(samples.get(i));
        }
   
      
   
        // Contains all the modes
        List<Double> modes = new ArrayList<Double>();
   
        double highmark = 0;
        for (Double x: tree) {
           double freq = Collections.frequency(list, x);
           if (freq == highmark) {
              modes.add(x);
           }
           if (freq > highmark) {
              modes.clear();
              modes.add(x);
              highmark = freq;
           } 
        }
      
        //Just return first.
        return modes.get(0);
      
   
    }

}
