package org.usfirst.frc.team2635.robot.model;

import java.util.HashMap;
import java.util.Map;

import org.usfirst.frc.team2635.robot.commands.DriveRotateMotionMagic;
import org.usfirst.frc.team2635.robot.commands.DriveStraightMotionMagic;

import edu.wpi.first.wpilibj.command.CommandGroup;

import java.util.ArrayList;

public class MotionProfileLibrary
{


	public static MotionParameters getRotationParameters(double targetAngle, 
														   double wheelRadiusInches,
														   double turnRadiusInches, 
														   double wheelSeparationInches,  
														   double rpm, 
														   boolean Clockwise, 
														   boolean rotateCenter)
	{
		double inchesPerRotation = wheelRadiusInches * 2 * Math.PI;
		
		double arcLengthInner;
		double archLengthOuter;
		double innerWheelRotations;
		double outerWheelRotations;
		
		if (rotateCenter)
		{			
			//To rotate around center.
			double radius = wheelSeparationInches/2.0;
			//radius is 1/2 of wheelSeparationInches
			//ArcLengh = radius * angle in radians
			
			arcLengthInner = radius *  (2*Math.PI)/360.0 * targetAngle;
			archLengthOuter = arcLengthInner;
			innerWheelRotations = arcLengthInner/inchesPerRotation;
			outerWheelRotations = archLengthOuter/inchesPerRotation;

		}
		else
		{	
			arcLengthInner = turnRadiusInches *  (2*Math.PI)/360.0 * targetAngle;
			archLengthOuter = (turnRadiusInches + wheelSeparationInches)  *  (2*Math.PI)/360.0 * targetAngle;
			innerWheelRotations = arcLengthInner/inchesPerRotation;
			outerWheelRotations = -archLengthOuter/inchesPerRotation;
		}
		
		
		double velocityRatio = Math.abs(outerWheelRotations/innerWheelRotations);
		
		double innerVelocity = rpm;
		double outerVelocity = rpm * velocityRatio;
		
		double innerAcceleration = 2 * innerVelocity;
		double outerAcceleration = 2 * outerVelocity;
		
		
		if (!Clockwise && !rotateCenter)
		{
			double tmpRotation = innerWheelRotations;
			innerWheelRotations = outerWheelRotations;
			outerWheelRotations = tmpRotation;
			
			double tmpAcceleration = innerAcceleration;
			innerAcceleration = outerAcceleration;
			outerAcceleration = tmpAcceleration;
			
			double tmpVelocity = innerVelocity;
			innerVelocity = outerVelocity;
			outerVelocity = tmpVelocity;
		}
		else if (!Clockwise && rotateCenter)
		{
			innerWheelRotations = -innerWheelRotations;
			outerWheelRotations = -outerWheelRotations;
		}
		
		
		MotionParameters rotationParams = new MotionParameters();
		rotationParams.rightAcceleration = innerAcceleration;
		rotationParams.leftAcceleration = outerAcceleration;
		rotationParams.rightVelocity     = innerVelocity;
		rotationParams.leftVelocity     = outerVelocity;
		rotationParams.rightWheelRotations = innerWheelRotations;
		rotationParams.leftWheelRotations = outerWheelRotations;

		return rotationParams;

//		System.out.println("innerVelocity:" + innerVelocity);
//		System.out.println("outerVelocity:" + outerVelocity);
//		
//		System.out.println("innerAcceleration:" + innerAcceleration);
//		System.out.println("outerAcceleration:" + outerAcceleration);
//		 
//		System.out.println("outerWheelRotations:" + outerWheelRotations);
//		System.out.println("innerWheelRotations:" + innerWheelRotations);
		

	}
	
	public static MotionParameters getDriveParameters(double wheelRadiusInches, double distanceInches, double rpm, boolean reverse)
	{
		double inchesPerRotation = wheelRadiusInches * 2 * Math.PI;
		
		//double arcLengthInner;
		//double archLengthOuter;
		double velocity = rpm;
		double acceleration = rpm * 2;
		double leftWheelRotations = -distanceInches/inchesPerRotation;
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
	
	public static CommandGroup getCenterGearPlacementSequence()
	{
		//1. Drive n Inches forward
		//2. Vision. Get Angle
		//   Rotate Angle
		//3. Vision. Get Distance
		//3. Drive N Inches
		//4. Activate Pneumatics 
		//DriveForward
		
		
		CommandGroup resultGroup = new CommandGroup();
		
		//DriveStraightMotionMagic cmd1 = new DriveStraightMotionMagic(100, 78.5, false);
		DriveStraightMotionMagic cmd1 = new DriveStraightMotionMagic(100, 24, false);
		resultGroup.addSequential(cmd1);
				
		
		double rpm = 200;
		double targetAngle = 90;
		double turnRadiusInche = 0;
		boolean clockwise = true;
		boolean rotateCenter = true;
		//new DriveRotateMotionMagic(rpm, targetAngle, turnRadiusInche, clockwise, rotateCenter));	

		return resultGroup;
		
	}
    	
	public CommandGroup getLeftGearPlacementSequence()
	{
		CommandGroup resultGroup = null;
		return resultGroup;
		
	}
	
	
	public CommandGroup getRightGearPlacementSequence()
	{
		CommandGroup resultGroup = null;
		return resultGroup;
		
	}
}
	
	

