package org.usfirst.frc.team2635.robot.model;

import java.util.HashMap;
import java.util.Map;
import java.util.ArrayList;

public class MotionProfileLibrary
{


	public static RotationParameters getRotationParameters(double targetAngle, 
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
		
		
		RotationParameters rotationParams = new RotationParameters();
		rotationParams.innerAcceleration = innerAcceleration;
		rotationParams.outerAcceleration = outerAcceleration;
		rotationParams.innerVelocity     = innerVelocity;
		rotationParams.outerVelocity     = outerVelocity;
		rotationParams.innerWheelRotations = innerWheelRotations;
		rotationParams.outerWheelRotations = outerWheelRotations;

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
	
	public static DriveParameters getDriveParameters(double wheelRadiusInches, double distanceInches, double rpm, boolean reverse)
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
		
		DriveParameters driveParams = new DriveParameters();
		driveParams.maxAcceleration = acceleration;
		driveParams.leftWheelRotations = leftWheelRotations;
		driveParams.rightWheelRotations = rightWheelRotations;
		driveParams.maxVelocity     = velocity;
		
		return driveParams;

	}	
		
}
	
	

