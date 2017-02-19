package org.usfirst.frc.team2635.robot.commands;

import org.usfirst.frc.team2635.robot.Robot;
import org.usfirst.frc.team2635.robot.model.Navx;
import org.usfirst.frc.team2635.robot.subsystems.Drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Drive using the navx based on the input from the camera.
 */
public class DriveCameraAnglePID extends Command {
	//degrees
	
	
    public DriveCameraAnglePID() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.drive);
    	requires(Robot.vision);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.drive.enableAnglePID();
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Double angle = Robot.vision.getAngleToBoiler();
    	if(angle != null)
    	{
    		Robot.drive.driveAnglePID(angle);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return Robot.drive.angleOnTarget();
    	
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drive.disableAnglePID();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.drive.disableAnglePID();
    }

}
