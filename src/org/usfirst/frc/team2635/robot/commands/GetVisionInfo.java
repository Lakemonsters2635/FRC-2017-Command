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
    	
		 Double modeAngle = modeit(angleSamples);
		 System.out.println("modeAngle: " + modeAngle);
		 
		 if (angleSamples.size() > 0)
		 {
			 visionParameters.AngleToTarget = modeit(angleSamples);
		 }
		 else
		 {
			 visionParameters.AngleToTarget = 0.0;
			 System.out.println("WARNING:Setting visionParameters.AngleToTarget to 0.0");
		 }
		 
		 if (distanceSamples.size() > 0)
		 {
			 visionParameters.DistanceToTarget = modeit(distanceSamples);
		 }
		 else
		 {
			 visionParameters.DistanceToTarget = 0.0;
			 System.out.println("ARNING:Setting visionParameters.DistanceToTarget to 0.0");
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
    
    double roundit(double num, double N) {
        double d = Math.log10(num);
        double power;
        if (num > 0) {
            d = Math.ceil(d);
            power = -(d-N);
        } else {
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
           
           System.out.println("samples[" + i + "]:" + samples.get(i) );
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
