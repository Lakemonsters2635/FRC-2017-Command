package org.usfirst.frc.team2635.robot.commands;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
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

public class GetVisionInfo2 extends Command{



	public SensorParameters visionParameters; 
	public String targetName;
	public boolean isFinished = false;
	
 
	
    public GetVisionInfo2(SensorParameters visionParams, String targetName) {
        // Use requires() here to declare subsystem dependencies

    	requires(Robot.light);
    	requires(Robot.vision);

    	this.visionParameters = visionParams;
    	this.targetName = targetName;

    	
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
	     	Robot.light.lightOn();    
	    	
	    	if (visionParameters != null)
	    	{
	    		visionParameters.AngleToTarget = null;
	    		visionParameters.DistanceToTarget = null;
	    		visionParameters.TargetAcquired = false;
	    	}
	    	System.out.println("Vision Initialized at "+ LocalDateTime.now());
	    	isFinished = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Double angle = null;
    	Double distance = null;
    	if (targetName == "Gear") {    	
    		Robot.vision.gearAim();
    		angle = Robot.vision.getAngleToGear();
    		distance = Robot.vision.getDistanceToGear();
    		if (angle!= null) {
    			
    		     //roundedAngle = roundit(angle, 2);
    		     //System.out.println("roundedAngle: " + roundedAngle);
   			
    		} else {
    			System.out.println("angle is NULL");
    		}
    		

    	} else if (targetName == "Boiler") {
    		Robot.vision.aim();
    		angle = Robot.vision.getAngleToBoiler();
    		distance = Robot.vision.getDistanceToBoiler();
    		
    	}


    	 String message = "";
    	 if (angle != null)
    	 {
    		 message += "Angle:" + angle;
    	 }
    	 if (distance != null)
    	 {
    		 message += "    Distance:" + distance;
    	 }
    	 if (message == "")
    	 {
    		 message = "Target not found";
    	 }
    	 visionParameters.AngleToTarget = angle;
    	 visionParameters.DistanceToTarget = distance;
    	 if(visionParameters.AngleToTarget != null) {
    		 visionParameters.TargetAcquired = true;
    	 }
    	 
		 Robot.vision.ViewShooter(message);
	      Robot.light.lightOff();
    	isFinished = true;
    }

    protected boolean isFinished() {
    	if(isFinished) {
    		System.out.println("-- GetVisionInfo2 isFinished --");
    	}
    	
        return isFinished;
    }

    // Called once after isFinished returns true
    protected void end() {
    	
    	
//    	String message = "";
//		 
//		 if (visionParameters.AngleToTarget > 0)
//		 {
//			 visionParameters.TargetAcquired = true;
//			 message = "Angle:" + visionParameters.AngleToTarget;
//		 }
//		 else
//		 {
//			 visionParameters.AngleToTarget = 0.0;
//			 visionParameters.TargetAcquired = false;
//			 System.out.println("WARNING:Setting visionParameters.AngleToTarget to 0.0");
//		 }
//		 
//		 if (visionParameters.DistanceToTarget > 0)
//		 {
//			 message += "  Distance:" + visionParameters.DistanceToTarget;
//		 }
//		 else
//		 {
//			 visionParameters.DistanceToTarget = 0.0;
//			 visionParameters.TargetAcquired = false;
//			 System.out.println("WARNING:Setting visionParameters.DistanceToTarget to 0.0");
//		 }	 
//	
//		 if (!visionParameters.TargetAcquired) {
//			 message = "Target not found.";
//		 }
//		 Robot.vision.ViewShooter(message);

	      System.out.println("Vision Ended at "+ LocalDateTime.now());


		 
		 
//		 if (!visionParameters.TargetAcquired)
//		 {
//			 CommandGroup parentGroup = this.getGroup();
//			 if (parentGroup != null)
//			 {
//				 System.out.println("*****************************************************");
//				 System.out.println("**** TARGET NOT ACQUIRED: ABORTING AUTONOMOUS.*******");
//				 System.out.println("*****************************************************");
//				 //Robot.vision.saveShooter();
//				 parentGroup.cancel();
//			 }
//			 
//		 }
		 
		 
		 
		 System.out.println("visionParameters.AngleToTarget: " + visionParameters.AngleToTarget + "\t visionParameters.DistanceToTarget:" + visionParameters.DistanceToTarget);
    	

    }




    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.light.lightOff();
    
    	if (visionParameters != null)
    	{
    		visionParameters.AngleToTarget = null;
    		visionParameters.DistanceToTarget = null;
    		visionParameters.TargetAcquired = false;
    	}
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
