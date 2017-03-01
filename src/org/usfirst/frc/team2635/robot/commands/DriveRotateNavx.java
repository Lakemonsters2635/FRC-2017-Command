package org.usfirst.frc.team2635.robot.commands;

import org.usfirst.frc.team2635.robot.Robot;
import org.usfirst.frc.team2635.robot.RobotMap;
import org.usfirst.frc.team2635.robot.model.MotionProfileLibrary;
import org.usfirst.frc.team2635.robot.model.RotationParameters;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveRotateNavx extends Command {

	public boolean wasPressed;
	public double heading;
    public DriveRotateNavx(double heading) {
        // Use requires() here to declare subsystem dependencies
       
       this.heading = heading;
       Robot.drive.setAnglePID(RobotMap.AIM_P, RobotMap.AIM_I, RobotMap.AIM_D);
      
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("DriveRotateNavX init");
      	//Robot.drive.updateMotionNavx(heading);
    	//Robot.drive.errNavxDrive = 0;
   
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        //Robot.drive.enableAnglePID();
//    	if (!wasPressed)
//    	{
//        	Robot.drive.updateMotionNavx(heading);
//	    	wasPressed = true;
//    	}
    	
    	System.out.println("DriveRotateNavX execute");
    	Robot.drive.updateMotionNavx(heading);
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	//FHE TODO:
//    	boolean done = Robot.drive.angleOnTarget();
//    	if (done){    		
//    		Robot.drive.disableAnglePID();
//    	}
//    	System.out.print("DriveRotateNavx is " + (done?"done":"not done"));
//    	return done;
    	return Robot.drive.motionNavxFinished(heading);
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
