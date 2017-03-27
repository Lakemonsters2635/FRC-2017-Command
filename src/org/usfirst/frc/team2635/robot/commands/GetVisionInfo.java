package org.usfirst.frc.team2635.robot.commands;
import edu.wpi.first.wpilibj.command.TimedCommand;
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.TreeSet;

import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.TreeSet;

import org.usfirst.frc.team2635.robot.Robot;
import org.usfirst.frc.team2635.robot.RobotMap;
import org.usfirst.frc.team2635.robot.model.SensorParameters;



/**
 *
 */

public class GetVisionInfo extends TimedCommand {


	ArrayList<Double>  angleSamples;
	ArrayList<Double>  distanceSamples;
	public SensorParameters visionParameters; 
	public String targetName;
	

	
    public GetVisionInfo(SensorParameters visionParams, String targetName, double timeout) {
        // Use requires() here to declare subsystem dependencies
    	super(timeout);
    	requires(Robot.light);
    	requires(Robot.vision);

    	this.visionParameters = visionParams;
    	this.targetName = targetName;

    	
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
     	Robot.light.lightOn();    
    	angleSamples = new ArrayList<Double>();
    	distanceSamples = new ArrayList<Double>();
    	if (visionParameters != null)
    	{
    		visionParameters.AngleToTarget = null;
    		visionParameters.DistanceToTarget = null;
    		visionParameters.TargetAcquired = false;
    	}
    	System.out.println("Vision Initialized at "+ LocalDateTime.now());

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Double angle = null;
    	Double distance = null;
    	Double roundedAngle = null;
    	Double roundedDistance = null;
    	if (targetName == "Gear") {    	
    		Robot.vision.gearAim();
    		angle = Robot.vision.getAngleToGear();
    		distance = Robot.vision.getDistanceToGear();
    		if (angle!= null) {
    		    roundedAngle = angle * 100;
    			roundedAngle = (double) Math.round(roundedAngle);
    			roundedAngle = roundedAngle/100;
    			
    		     //roundedAngle = roundit(angle, 2);
    		     //System.out.println("roundedAngle: " + roundedAngle);
   				angleSamples.add(roundedAngle);
    		} else {
    			System.out.println("angle is NULL");
    		}
    		
    		if (distance!= null){
    			roundedDistance = distance * 100;
    			roundedDistance = (double) Math.round(roundedDistance);
    			roundedDistance = roundedDistance/100;
    			
    			distanceSamples.add(roundedDistance);
    		}
    	} else if (targetName == "Boiler") {
    		Robot.vision.aim();
    		angle = Robot.vision.getAngleToBoiler();
    		distance = Robot.vision.getDistanceToBoiler();
    		
    		if (angle!= null) {
    			angleSamples.add(angle);
    		}
    		if (distance!= null) {
    			distanceSamples.add(distance);
    		}
    	}


    	
    }



    // Called once after isFinished returns true
    protected void end() {
    	

		 
		 if (angleSamples.size() > 0)
		 {
			 visionParameters.AngleToTarget = modeit(angleSamples, "Angle");
			 visionParameters.TargetAcquired = true;
		 }
		 else
		 {
			 visionParameters.AngleToTarget = 0.0;
			 visionParameters.TargetAcquired = false;
			 System.out.println("WARNING:Setting visionParameters.AngleToTarget to 0.0");
		 }
		 
		 if (distanceSamples.size() > 0)
		 {
			 visionParameters.DistanceToTarget = modeit(distanceSamples, "Distance");
		 }
		 else
		 {
			 visionParameters.DistanceToTarget = 0.0;
			 visionParameters.TargetAcquired = false;
			 System.out.println("WARNING:Setting visionParameters.DistanceToTarget to 0.0");
		 }	 
	
	    	angleSamples.clear();
	    	distanceSamples.clear();

	    	 System.out.println("Vision Ended at "+ LocalDateTime.now());
	    	Robot.light.lightOff();
		 
		 System.out.println("visionParameters.AngleToTarget: " + visionParameters.AngleToTarget + "\t visionParameters.DistanceToTarget:" + visionParameters.DistanceToTarget);
    	

    }




    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.light.lightOff();
    	angleSamples.clear();
    	distanceSamples.clear();  
    	System.out.println("Vision Interrupted at "+ LocalDateTime.now());
    }
    
    
    Double modeit(ArrayList<Double> samples, String collectionName){
    	
    	 
        // list of all the numbers 
        List<Double> list = new ArrayList<Double>();
   
        // list of all the numbers with no duplicates
        TreeSet<Double> tree = new TreeSet<Double>();
   
        for (int i = 0; i < samples.size(); i++) {
           list.add(samples.get(i));
           tree.add(samples.get(i));
           
   			if (RobotMap.DEBUG_DETAIL)
   			{
   				System.out.println(collectionName + " samples[" + i + "]:" + samples.get(i) );
   			}
           
           
           
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
        if (modes.size() > 0)
        {
        	return modes.get(0);
        }
        else
        {
        	return 0.0;
        }
        
    }

}
