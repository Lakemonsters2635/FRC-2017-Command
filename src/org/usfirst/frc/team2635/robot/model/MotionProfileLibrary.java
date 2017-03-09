package org.usfirst.frc.team2635.robot.model;

import java.util.HashMap;
import java.util.Map;

import org.usfirst.frc.team2635.robot.Robot;
import org.usfirst.frc.team2635.robot.commands.DeliverGearBackwards;
import org.usfirst.frc.team2635.robot.commands.DeliverGearForward;
import org.usfirst.frc.team2635.robot.commands.DriveCameraAnglePID;
import org.usfirst.frc.team2635.robot.commands.DriveRotateMotionMagic;
import org.usfirst.frc.team2635.robot.commands.DriveStraightMotionMagic;
import org.usfirst.frc.team2635.robot.commands.GetVisionInfo;
import org.usfirst.frc.team2635.robot.commands.MotionCommandGroup;
import org.usfirst.frc.team2635.robot.commands.UltrasonicCommand;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

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
		double distance1 = 40;
		double distance2 = 67.86 - distance1;
		double velocity = 50;
		double rpm = 100;

		MotionCommandGroup resultGroup = new MotionCommandGroup();
		
		//DriveStraightMotionMagic cmd1 = new DriveStraightMotionMagic(100, 78.5, false);
		DriveStraightMotionMagic drive1 = new DriveStraightMotionMagic(velocity, distance1, false);
		DriveStraightMotionMagic drive2 = new DriveStraightMotionMagic(velocity, distance2, false);
				
		DeliverGearForward gearForward = new DeliverGearForward();
		DeliverGearForward gearForward2 = new DeliverGearForward();

		
		WaitCommand waitCmd = new WaitCommand(1);
		WaitCommand waitCmd2 = new WaitCommand(1);
		WaitCommand waitCmd3 = new WaitCommand(1);
		WaitCommand waitCmd4 = new WaitCommand(1);
		
		String targetName = "Gear";
		VisionParameters visionParams = new VisionParameters(null,null);
		GetVisionInfo visionCmd1= new GetVisionInfo(visionParams, targetName,3);
		
		//UltrasonicParameters ultrasonicParams = new UltrasonicParameters(null, null);
		//UltrasonicCommand ultrasonicCmd1 = new UltrasonicCommand(ultrasonicParams);
		
		DeliverGearBackwards gearBackward = new DeliverGearBackwards();
		DeliverGearBackwards gearBackward2 = new DeliverGearBackwards();
		
		DriveStraightMotionMagic driveBackwards = new DriveStraightMotionMagic(velocity, distance1 + distance2, true);
		
		//resultGroup.addSequential(ultrasonicCmd1);
		double turnRadiusInches = 0;
		boolean clockwise = true;
		boolean rotateCenter = true;

		DriveRotateMotionMagic rotateToGearPegCmd = new DriveRotateMotionMagic(rpm, 0, turnRadiusInches, clockwise, rotateCenter, visionParams);	
		
		resultGroup.addSequential(drive1);
		resultGroup.addSequential(visionCmd1);
		resultGroup.addSequential(rotateToGearPegCmd);
		resultGroup.addSequential(drive2);
		resultGroup.addSequential(gearForward);
		resultGroup.addSequential(waitCmd);
		resultGroup.addSequential(gearBackward);
		resultGroup.addSequential(waitCmd2);
		resultGroup.addSequential(gearForward2);
		resultGroup.addSequential(waitCmd3);
		resultGroup.addSequential(gearBackward2);
		resultGroup.addSequential(waitCmd4);
		resultGroup.addSequential(driveBackwards);
		//

		return resultGroup;
		
	}
    	

	
	
	public static MotionCommandGroup getLeftGearPlacementSequence()
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
		
		VisionParameters visionParams = new VisionParameters(null,null);
		MotionCommandGroup resultGroup = new MotionCommandGroup();
		//resultGroup.doesRequire(Robot.drive);
		
		double straightVelocity = 50;
		//DriveStraightMotionMagic cmd1 = new DriveStraightMotionMagic(100, 78.5, false);
		DriveStraightMotionMagic drive1 = new DriveStraightMotionMagic(straightVelocity, drive1Distance, false);
		
		
		double rpm = 100;
		double targetAngle = 60;
		double turnRadiusInche = 0;
		boolean clockwise = true;
		boolean rotateCenter = true;
		
		DriveRotateMotionMagic rotateCmd = new DriveRotateMotionMagic(rpm, targetAngle, turnRadiusInche, clockwise, rotateCenter, visionParams);
		DriveRotateMotionMagic rotateCmdCCW = new DriveRotateMotionMagic(rpm, targetAngle, turnRadiusInche, false, rotateCenter, visionParams);

		
		
		//FHE TODO: 18 inches might not be right.
		DriveStraightMotionMagic drive2 = new DriveStraightMotionMagic(straightVelocity, 36, false);
				
		DeliverGearForward gearForward = new DeliverGearForward();
		DeliverGearForward gearForward2 = new DeliverGearForward();

		
		WaitCommand waitCmd = new WaitCommand(1);
		WaitCommand waitCmd2 = new WaitCommand(1);
		WaitCommand waitCmd3 = new WaitCommand(1);
		WaitCommand waitCmd4 = new WaitCommand(1);
		
		String targetName = "Gear";
		//VisionParameters visionParams = new VisionParameters(null,null);
		GetVisionInfo visionCmd1= new GetVisionInfo(visionParams, targetName,3);
		
		DeliverGearBackwards gearBackward = new DeliverGearBackwards();
		DeliverGearBackwards gearBackward2 = new DeliverGearBackwards();
		
		DriveStraightMotionMagic shortDriveBackwards = new DriveStraightMotionMagic(straightVelocity, 36, true);
		
		double turnRadiusInches = 0;
		
		DriveStraightMotionMagic driveBackwards = new DriveStraightMotionMagic(straightVelocity,  114.5, true);
		
		DriveRotateMotionMagic rotateToGearPegCmd = new DriveRotateMotionMagic(rpm, 0, turnRadiusInches, clockwise, rotateCenter, visionParams);	

		

		
		resultGroup.addSequential(drive1);
		resultGroup.addSequential(rotateCmd);
		resultGroup.addSequential(visionCmd1);
		resultGroup.addSequential(rotateToGearPegCmd);
		resultGroup.addSequential(drive2);
		resultGroup.addSequential(gearForward);
		resultGroup.addSequential(waitCmd);
		resultGroup.addSequential(gearBackward);
		resultGroup.addSequential(waitCmd2);
		resultGroup.addSequential(gearForward2);
		resultGroup.addSequential(waitCmd3);
		resultGroup.addSequential(gearBackward2);
		resultGroup.addSequential(waitCmd4);
		resultGroup.addSequential(shortDriveBackwards);
		//resultGroup.addSequential(rotateCmdCCW);
		//resultGroup.addSequential(driveBackwards);
		//new DriveRotateMotionMagic(rpm, targetAngle, turnRadiusInche, clockwise, rotateCenter));	
		
		//resultGroup.addSequential(cmd2);
		
		//FHE:TODO. This Camera Command Looks for the boiler. We need a Vision solution for Gear Placement.
		//		DriveCameraAnglePID findPegCmd = new DriveCameraAnglePID();
		//		resultGroup.addSequential(findPegCmd);		
		
		return resultGroup;
		
	}
	
	
	public static MotionCommandGroup getRightGearPlacementSequence()
	{
double drive1Distance = 75.385;
		
		VisionParameters visionParams = new VisionParameters(null,null);
		MotionCommandGroup resultGroup = new MotionCommandGroup();
		//resultGroup.doesRequire(Robot.drive);
		
		double straightVelocity = 50;
		//DriveStraightMotionMagic cmd1 = new DriveStraightMotionMagic(100, 78.5, false);
		DriveStraightMotionMagic drive1 = new DriveStraightMotionMagic(straightVelocity, drive1Distance, false);
		
		
		double rpm = 100;
		double targetAngle = -60;
		double turnRadiusInche = 0;
		boolean clockwise = true;
		boolean rotateCenter = true;
		
		DriveRotateMotionMagic rotateCmd = new DriveRotateMotionMagic(rpm, targetAngle, turnRadiusInche, clockwise, rotateCenter, visionParams);
		DriveRotateMotionMagic rotateCmdCCW = new DriveRotateMotionMagic(rpm, targetAngle, turnRadiusInche, false, rotateCenter, visionParams);

		
		
		//FHE TODO: 18 inches might not be right.
		DriveStraightMotionMagic drive2 = new DriveStraightMotionMagic(straightVelocity, 36, false);
				
		DeliverGearForward gearForward = new DeliverGearForward();
		DeliverGearForward gearForward2 = new DeliverGearForward();

		
		WaitCommand waitCmd = new WaitCommand(1);
		WaitCommand waitCmd2 = new WaitCommand(1);
		WaitCommand waitCmd3 = new WaitCommand(1);
		WaitCommand waitCmd4 = new WaitCommand(1);
		
		String targetName = "Gear";
		//VisionParameters visionParams = new VisionParameters(null,null);
		GetVisionInfo visionCmd1= new GetVisionInfo(visionParams, targetName,3);
		
		DeliverGearBackwards gearBackward = new DeliverGearBackwards();
		DeliverGearBackwards gearBackward2 = new DeliverGearBackwards();
		
		DriveStraightMotionMagic shortDriveBackwards = new DriveStraightMotionMagic(straightVelocity, 36, true);
		
		double turnRadiusInches = 0;
		
		DriveStraightMotionMagic driveBackwards = new DriveStraightMotionMagic(straightVelocity,  114.5, true);
		
		DriveRotateMotionMagic rotateToGearPegCmd = new DriveRotateMotionMagic(rpm, 0, turnRadiusInches, clockwise, rotateCenter, visionParams);	

		

		
		resultGroup.addSequential(drive1);
		resultGroup.addSequential(rotateCmd);
		resultGroup.addSequential(visionCmd1);
		resultGroup.addSequential(rotateToGearPegCmd);
		resultGroup.addSequential(drive2);
		resultGroup.addSequential(gearForward);
		resultGroup.addSequential(waitCmd);
		resultGroup.addSequential(gearBackward);
		resultGroup.addSequential(waitCmd2);
		resultGroup.addSequential(gearForward2);
		resultGroup.addSequential(waitCmd3);
		resultGroup.addSequential(gearBackward2);
		resultGroup.addSequential(waitCmd4);
		resultGroup.addSequential(shortDriveBackwards);
		//resultGroup.addSequential(rotateCmdCCW);
		//resultGroup.addSequential(driveBackwards);
		//new DriveRotateMotionMagic(rpm, targetAngle, turnRadiusInche, clockwise, rotateCenter));	
		
		//resultGroup.addSequential(cmd2);
		
		//FHE:TODO. This Camera Command Looks for the boiler. We need a Vision solution for Gear Placement.
		//		DriveCameraAnglePID findPegCmd = new DriveCameraAnglePID();
		//		resultGroup.addSequential(findPegCmd);		
		
		return resultGroup;
		
	}
}
	
	

