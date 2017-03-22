package org.usfirst.frc.team2635.robot.model;

import java.util.HashMap;
import java.util.Map;

import org.usfirst.frc.team2635.robot.Robot;
import org.usfirst.frc.team2635.robot.RobotMap;
import org.usfirst.frc.team2635.robot.commands.DeliverGearBackwards;
import org.usfirst.frc.team2635.robot.commands.DeliverGearForward;
import org.usfirst.frc.team2635.robot.commands.DriveCameraAnglePID;
import org.usfirst.frc.team2635.robot.commands.DriveRotateMotionMagic;
import org.usfirst.frc.team2635.robot.commands.DriveStraightMotionMagic;
import org.usfirst.frc.team2635.robot.commands.GetVisionInfo;
import org.usfirst.frc.team2635.robot.commands.MotionCommandGroup;
import org.usfirst.frc.team2635.robot.commands.UltrasonicCommand;
import org.usfirst.frc.team2635.robot.model.SensorParameters;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;

import java.util.ArrayList;

public class MotionProfileLibrary
{


	public static MotionParameters getArcRotationParameters(double targetAngle, 
														   double wheelRadiusInches,
														   double turnRadiusInches, 
														   double wheelSeparationInches,  
														   double rpm, 
														   boolean Clockwise, 
														   boolean rotateCenter)
	{
		double inchesPerRotation = wheelRadiusInches * 2 * Math.PI;
		
		double arcLengthRight;
		double archLengthLeft;
		double rightWheelRotations;
		double leftWheelRotations;
		
		if (rotateCenter)
		{			
			//To rotate around center.
			double radius = wheelSeparationInches/2.0;
			//radius is 1/2 of wheelSeparationInches
			//ArcLengh = radius * angle in radians
			
			arcLengthRight = radius *  (2*Math.PI)/360.0 * targetAngle;
			archLengthLeft = arcLengthRight;
			rightWheelRotations = arcLengthRight/inchesPerRotation;
			leftWheelRotations = archLengthLeft/inchesPerRotation;

		}
		else
		{	
			arcLengthRight = turnRadiusInches *  (2*Math.PI)/360.0 * targetAngle;
			archLengthLeft = (turnRadiusInches + wheelSeparationInches)  *  (2*Math.PI)/360.0 * targetAngle;
			rightWheelRotations = arcLengthRight/inchesPerRotation;
			leftWheelRotations = -archLengthLeft/inchesPerRotation;
		}
		
		
		double velocityRatio = Math.abs(leftWheelRotations/rightWheelRotations);
		
		double rightVelocity = rpm;
		double leftVelocity = rpm * velocityRatio;
		
		double rightAcceleration =  rightVelocity;
		double leftAcceleration =  leftVelocity;
		
		
		if (!Clockwise && !rotateCenter)
		{
			double tmpRotation = rightWheelRotations;
			rightWheelRotations = leftWheelRotations;
			leftWheelRotations = tmpRotation;
			
			double tmpAcceleration = rightAcceleration;
			rightAcceleration = leftAcceleration;
			leftAcceleration = tmpAcceleration;
			
			double tmpVelocity = rightVelocity;
			rightVelocity = leftVelocity;
			leftVelocity = tmpVelocity;
		}
		else if (Clockwise && rotateCenter)
		{
			rightWheelRotations = -rightWheelRotations;
			leftWheelRotations = -leftWheelRotations;
		}
		
		
		MotionParameters rotationParams = new MotionParameters();
		rotationParams.rightAcceleration = rightAcceleration;
		rotationParams.leftAcceleration = leftAcceleration;
		rotationParams.rightVelocity     = rightVelocity;
		rotationParams.leftVelocity     = leftVelocity;
		rotationParams.rightWheelRotations = rightWheelRotations;
		rotationParams.leftWheelRotations = leftWheelRotations;

		return rotationParams;

//		System.out.println("rightVelocity:" + rightVelocity);
//		System.out.println("leftVelocity:" + leftVelocity);
//		
//		System.out.println("rightAcceleration:" + rightAcceleration);
//		System.out.println("leftAcceleration:" + leftAcceleration);
//		 
//		System.out.println("leftWheelRotations:" + leftWheelRotations);
//		System.out.println("rightWheelRotations:" + rightWheelRotations);
		

	}
	
	
	public static MotionParameters getRotationParameters(double targetAngle,    double wheelRadiusInches, double wheelSeparationInches,  double rpm)	
	{
			double inchesPerRotation = wheelRadiusInches * 2 * Math.PI;
			
			double arcLengthRight;
			double archLengthLeft;
			double rightWheelRotations;
			double leftWheelRotations;
			
			
			//To rotate around center.
			double radius = wheelSeparationInches/2.0;
			//radius is 1/2 of wheelSeparationInches
			//ArcLengh = radius * angle in radians
			
			arcLengthRight = radius *  (2*Math.PI)/360.0 * targetAngle;
			archLengthLeft = arcLengthRight;
			rightWheelRotations = -arcLengthRight/inchesPerRotation;
			leftWheelRotations = -archLengthLeft/inchesPerRotation;
			
		
				
			
			double velocityRatio = Math.abs(leftWheelRotations/rightWheelRotations);
			
			double rightVelocity = rpm;
			double leftVelocity = rpm * velocityRatio;
			
			double rightAcceleration =  rightVelocity;
			double leftAcceleration =  leftVelocity;
		
			
			
			MotionParameters rotationParams = new MotionParameters();
			rotationParams.rightAcceleration = rightAcceleration;
			rotationParams.leftAcceleration = leftAcceleration;
			rotationParams.rightVelocity     = rightVelocity;
			rotationParams.leftVelocity     = leftVelocity;
			rotationParams.rightWheelRotations = rightWheelRotations;
			rotationParams.leftWheelRotations = leftWheelRotations;
			
			return rotationParams;
			
			//System.out.println("rightVelocity:" + rightVelocity);
			//System.out.println("leftVelocity:" + leftVelocity);
			//
			//System.out.println("rightAcceleration:" + rightAcceleration);
			//System.out.println("leftAcceleration:" + leftAcceleration);
			//
			//System.out.println("leftWheelRotations:" + leftWheelRotations);
			//System.out.println("rightWheelRotations:" + rightWheelRotations);
			
			
			}

	
	public static MotionParameters getDriveParameters(double wheelRadiusInches, double distanceInches, double rpm, boolean reverse)
	{
		

		double inchesPerRotation = wheelRadiusInches * 2 * Math.PI;
		
		//double arcLengthInner;
		//double archLengthOuter;
		double velocity = rpm;
		double acceleration = rpm * 1;
		
		//FOR COMPETITION BOT DO THE FOLLOWING
		double leftWheelRotations = -distanceInches/inchesPerRotation;
		//END COMPETITION BOT
		
		double rightWheelRotations = distanceInches/inchesPerRotation;
		
		if (reverse)
		{			
			leftWheelRotations = -leftWheelRotations;
			rightWheelRotations = -rightWheelRotations;
		}
		
		MotionParameters driveParams = new MotionParameters();
		driveParams.leftAcceleration = acceleration;
		driveParams.rightAcceleration = acceleration;
		driveParams.leftWheelRotations = leftWheelRotations;
		driveParams.rightWheelRotations = rightWheelRotations;
		driveParams.leftVelocity     = velocity;
		driveParams.rightVelocity     = velocity;
		return driveParams;

	}	
	
	public static MotionCommandGroup getCenterGearPlacementSequence()
	{
		//1. Drive n Inches forward
		//2. Vision. Get Angle (NOT DONE YET)
		//   Rotate Angle (NOT DONE YET)
		//3. Vision. Get Distance (NOT DONE YET)
		//3. Drive N Inches (NOT DONE YET)
		//4. Activate Pneumatics 
		//Wait N seconds for Pneumatics
		//Reverse Pneumatics
		//Wait N seconds for reveres Pneumatics
		
		//DriveForward
		//double distance = 37.699111843077518861551720599354; //Two Rotations
		//double distance1 = 40;
		
		double distance1 =  67.86;
		double velocity = 300;
		double rpm = 300;

		MotionCommandGroup resultGroup = new MotionCommandGroup();
		
		//DriveStraightMotionMagic cmd1 = new DriveStraightMotionMagic(100, 78.5, false);
		DriveStraightMotionMagic drive1 = new DriveStraightMotionMagic(velocity, distance1, false);

		DeliverGearForward gearForward = new DeliverGearForward(RobotMap.GEAR_DELIVERY_TIMEOUT);
		DeliverGearForward gearForward2 = new DeliverGearForward(RobotMap.GEAR_DELIVERY_TIMEOUT);

		//String targetName = "Gear";
		//VisionParameters visionParams = new VisionParameters(null,null);
		//GetVisionInfo visionCmd1= new GetVisionInfo(visionParams, targetName,3);
		
		//UltrasonicParameters ultrasonicParams = new UltrasonicParameters(null, null);
		//UltrasonicCommand ultrasonicCmd1 = new UltrasonicCommand(ultrasonicParams);
		

		DeliverGearBackwards gearBackward = new DeliverGearBackwards(RobotMap.GEAR_DELIVERY_TIMEOUT);
		DeliverGearBackwards gearBackward2 = new DeliverGearBackwards(RobotMap.GEAR_DELIVERY_TIMEOUT);
		DriveStraightMotionMagic driveBackwards = new DriveStraightMotionMagic(velocity, distance1, true);
		//resultGroup.addSequential(visionCmd1);
		//resultGroup.addSequential(rotateToGearPegCmd);
		resultGroup.addSequential(drive1);

		
		resultGroup.addSequential(gearForward);
		resultGroup.addSequential(gearBackward);
		resultGroup.addSequential(gearForward2);
		resultGroup.addSequential(gearBackward2);
		resultGroup.addSequential(driveBackwards);
		//

		return resultGroup;
		
	}
    	


	
	public static MotionCommandGroup getLeftGearPlacementSequence()
	{
		
		double drive1Distance = 75.385;
		double distanceAfter60degreeRotation = 31.177;
		
		SensorParameters visionParams = new SensorParameters(null,null);
		SensorParameters ultrasonicParams = new SensorParameters(null, null);
		
		MotionCommandGroup resultGroup = new MotionCommandGroup();
		
		

		double straightVelocity = 300;
		DriveStraightMotionMagic drive1 = new DriveStraightMotionMagic(straightVelocity, drive1Distance, false);
		
		double rpm = 300;
		double targetAngle = 60;
		DriveRotateMotionMagic rotateCmd = new DriveRotateMotionMagic(rpm, targetAngle);
	
		//drive after rotate.
		//Actual distance should be 31.177, but we want to stop for sonar reading.
		//Stop 20 inches short and use sonar
		visionParams.DistanceAdjustment = -20;
		DriveStraightMotionMagic drive2 = new DriveStraightMotionMagic(straightVelocity,visionParams);
		UltrasonicCommand ultrasonicCmd1 = new UltrasonicCommand(ultrasonicParams, 0.25);
		
		
		straightVelocity = 100; //slow down for final approach
		DriveStraightMotionMagic driveOnSonar = new DriveStraightMotionMagic(straightVelocity, ultrasonicParams);

		
		DeliverGearForward gearForward = new DeliverGearForward(RobotMap.GEAR_DELIVERY_TIMEOUT);
		DeliverGearForward gearForward2 = new DeliverGearForward(RobotMap.GEAR_DELIVERY_TIMEOUT);
		
		DriveStraightMotionMagic shortDriveBackwards = new DriveStraightMotionMagic(straightVelocity, 31.177, true);
		
		String targetName = "Gear";
		GetVisionInfo visionCmd1= new GetVisionInfo(visionParams, targetName,1); //FHE: Is two seconds for vision right?
		
		DriveRotateMotionMagic rotateBasedOnVision = new DriveRotateMotionMagic(rpm,  visionParams);		
		
		DeliverGearBackwards gearBackward = new DeliverGearBackwards(RobotMap.GEAR_DELIVERY_TIMEOUT);
		DeliverGearBackwards gearBackward2 = new DeliverGearBackwards(RobotMap.GEAR_DELIVERY_TIMEOUT);
			

		
		resultGroup.addSequential(drive1);
		resultGroup.addSequential(rotateCmd);
		resultGroup.addSequential(visionCmd1);
		resultGroup.addSequential(rotateBasedOnVision);
		resultGroup.addSequential(drive2);
		resultGroup.addSequential(ultrasonicCmd1);
		resultGroup.addSequential(driveOnSonar);
		resultGroup.addSequential(gearForward);
		resultGroup.addSequential(gearBackward);
		resultGroup.addSequential(gearForward2);
		resultGroup.addSequential(gearBackward2);
		resultGroup.addSequential(shortDriveBackwards);

		
		//resultGroup.addSequential(cmd2);
		
		//FHE:TODO. This Camera Command Looks for the boiler. We need a Vision solution for Gear Placement.
		//		DriveCameraAnglePID findPegCmd = new DriveCameraAnglePID();
		//		resultGroup.addSequential(findPegCmd);		
		
		return resultGroup;
		
	}
	
	public static MotionCommandGroup getSimpleLeftGearPlacementSequence()
	{
		
		//1. Drive n Inches forward
		//2. Vision. Get Angle (NOT DONE YET)
		//   Rotate Angle (NOT DONE YET)
		//3. Vision. Get Distance (NOT DONE YET)
		//3. Drive N Inches (NOT DONE YET)
		//4. Activate Pneumatics 
		//Wait N seconds for Pneumatics
		//Reverse Pneumatics
		//Wait N seconds for reveres Pneumatics
		
		//DriveForward
		//public ConfigurationInfo = new ConfigurationInfo();
		double drive1Distance = 75.385;
		double driveDistanceAfter60degreeRotation = 31.177;
		
		SensorParameters visionParams = new SensorParameters(null,null);
		MotionCommandGroup resultGroup = new MotionCommandGroup();
		
		
		double velocity = 250;
		DriveStraightMotionMagic drive1 = new DriveStraightMotionMagic(velocity, drive1Distance, false);
		
		
	
		double targetAngle = 60;

		DriveRotateMotionMagic rotateCmd = new DriveRotateMotionMagic(velocity, targetAngle);
		
		//drive after rotate.
		//Actual distance is 31.177, but we want to stop for sonar reading.
		
		DriveStraightMotionMagic drive2 = new DriveStraightMotionMagic(velocity,driveDistanceAfter60degreeRotation, false);
		DeliverGearForward gearForward = new DeliverGearForward(RobotMap.GEAR_DELIVERY_TIMEOUT);
		DeliverGearForward gearForward2 = new DeliverGearForward(RobotMap.GEAR_DELIVERY_TIMEOUT);
		
		DriveStraightMotionMagic shortDriveBackwards = new DriveStraightMotionMagic(velocity, driveDistanceAfter60degreeRotation, true);		
		DeliverGearBackwards gearBackward = new DeliverGearBackwards(RobotMap.GEAR_DELIVERY_TIMEOUT);
		DeliverGearBackwards gearBackward2 = new DeliverGearBackwards(RobotMap.GEAR_DELIVERY_TIMEOUT);
			

		
		resultGroup.addSequential(drive1);
		resultGroup.addSequential(rotateCmd);
		resultGroup.addSequential(drive2);
		resultGroup.addSequential(gearForward);
		resultGroup.addSequential(gearBackward);
		resultGroup.addSequential(gearForward2);
		resultGroup.addSequential(gearBackward2);	
		
		return resultGroup;
		
	}
	
	
	public static MotionCommandGroup doNothing()
	{
		
		MotionCommandGroup resultGroup = new MotionCommandGroup();
		
		WaitCommand waitCmd = new WaitCommand(1);
		
		resultGroup.addSequential(waitCmd);
		return resultGroup;
	
	}
	
	
	
	public static MotionCommandGroup getRightGearPlacementSequence()
	{
		
		double drive1Distance = 75.385;
		double distanceAfter60degreeRotation = 31.177;
		
		SensorParameters visionParams = new SensorParameters(null,null);
		SensorParameters ultrasonicParams = new SensorParameters(null, null);
		
		MotionCommandGroup resultGroup = new MotionCommandGroup();
		
		

		double straightVelocity = 300;
		DriveStraightMotionMagic drive1 = new DriveStraightMotionMagic(straightVelocity, drive1Distance, false);
		
		double rpm = 300;
		double targetAngle = -60;
		DriveRotateMotionMagic rotateCmd = new DriveRotateMotionMagic(rpm, targetAngle);
	
		//drive after rotate.
		//Actual distance should be 31.177, but we want to stop for sonar reading.
		//Stop 20 inches short and use sonar
		visionParams.DistanceAdjustment = -20;
		DriveStraightMotionMagic drive2 = new DriveStraightMotionMagic(straightVelocity,visionParams);
		UltrasonicCommand ultrasonicCmd1 = new UltrasonicCommand(ultrasonicParams, 0.25);
		
		
		straightVelocity = 100; //slow down for final approach
		DriveStraightMotionMagic driveOnSonar = new DriveStraightMotionMagic(straightVelocity, ultrasonicParams);

		
		DeliverGearForward gearForward = new DeliverGearForward(RobotMap.GEAR_DELIVERY_TIMEOUT);
		DeliverGearForward gearForward2 = new DeliverGearForward(RobotMap.GEAR_DELIVERY_TIMEOUT);
		
		DriveStraightMotionMagic shortDriveBackwards = new DriveStraightMotionMagic(straightVelocity, 31.177, true);
		
		String targetName = "Gear";
		GetVisionInfo visionCmd1= new GetVisionInfo(visionParams, targetName,1); //FHE: Is two seconds for vision right?
		
		DriveRotateMotionMagic rotateBasedOnVision = new DriveRotateMotionMagic(rpm,  visionParams);		
		
		DeliverGearBackwards gearBackward = new DeliverGearBackwards(RobotMap.GEAR_DELIVERY_TIMEOUT);
		DeliverGearBackwards gearBackward2 = new DeliverGearBackwards(RobotMap.GEAR_DELIVERY_TIMEOUT);
			

		
		resultGroup.addSequential(drive1);
		resultGroup.addSequential(rotateCmd);
		resultGroup.addSequential(visionCmd1);
		resultGroup.addSequential(rotateBasedOnVision);
		resultGroup.addSequential(drive2);
		resultGroup.addSequential(ultrasonicCmd1);
		resultGroup.addSequential(driveOnSonar);
		resultGroup.addSequential(gearForward);
		resultGroup.addSequential(gearBackward);
		resultGroup.addSequential(gearForward2);
		resultGroup.addSequential(gearBackward2);
		resultGroup.addSequential(shortDriveBackwards);

		
		//resultGroup.addSequential(cmd2);
		
		//FHE:TODO. This Camera Command Looks for the boiler. We need a Vision solution for Gear Placement.
		//		DriveCameraAnglePID findPegCmd = new DriveCameraAnglePID();
		//		resultGroup.addSequential(findPegCmd);		
		
		return resultGroup;
		
	}
		

	
	
	public static MotionCommandGroup RotateTest()
	{
		MotionCommandGroup resultGroup = new MotionCommandGroup();
		SensorParameters visionParams = new SensorParameters(null,null);
		double rpm = 100;
		double targetAngle = 60;

		DriveRotateMotionMagic rotateCmd = new DriveRotateMotionMagic(rpm, targetAngle);
		
		resultGroup.addSequential(rotateCmd);

		
		return resultGroup;
	}
	
	public static MotionCommandGroup visionTestSequence()
	{

		DriveRotateMotionMagic rotate1 = new DriveRotateMotionMagic(250, 0);
		
		
		SensorParameters visionParams = new SensorParameters(null,null);
		SensorParameters ultrasonicParams = new SensorParameters(null, null);
		double velocity = 250;
		String targetName = "Gear";
		GetVisionInfo visionCmd1= new GetVisionInfo(visionParams, targetName,2); //FHE: Is two seconds for vision right?
		DriveRotateMotionMagic rotateBasedOnVision = new DriveRotateMotionMagic(velocity,  visionParams);	
		//	UltrasonicCommand ultrasonicCmd1 = new UltrasonicCommand(ultrasonicParams, 1);
		DriveStraightMotionMagic drive1 = new DriveStraightMotionMagic(velocity, ultrasonicParams);
		MotionCommandGroup resultGroup = new MotionCommandGroup();

		resultGroup.addSequential(rotate1);
		resultGroup.addSequential(visionCmd1);
		resultGroup.addSequential(rotateBasedOnVision);
		//resultGroup.addSequential(ultrasonicCmd1);
		//resultGroup.addSequential(drive1);
		return resultGroup;
		
	}
	
	public static MotionCommandGroup rotateTestSequence()
	{
		SensorParameters visionParams = new SensorParameters(-1.0,1.0);

		//
		
		MotionCommandGroup resultGroup = new MotionCommandGroup();
		double rpm = 300;

		
		String targetName = "Gear";
		 //FHE: Is two seconds for vision right?
		
		DriveRotateMotionMagic rotateBasedOnVision = new DriveRotateMotionMagic(rpm,  visionParams);		


		
		resultGroup.addSequential(rotateBasedOnVision);
		
		return resultGroup;
		
	}
	
}
	
	

