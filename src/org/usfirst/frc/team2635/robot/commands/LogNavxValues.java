package org.usfirst.frc.team2635.robot.commands;

import org.usfirst.frc.team2635.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class LogNavxValues extends Command {

	public boolean wasPressed;
	
    public LogNavxValues() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
//    	SmartDashboard.putNumber("Navx angle" ,Robot.drive.getNavx().getAngle());
//    	SmartDashboard.putNumber("Navx unwrapped angle", Robot.drive.getNavx().getUnwrappedAngle());
    	
    	if (!wasPressed) {
	    	System.out.println("Navx angle: " + Robot.drive.getNavx().getAngle());
	    	System.out.println("Navx heading: " + Robot.drive.getNavx().getHeading());
	    	float[] displacement = Robot.drive.getNavx().getDisplacement();
	    	System.out.println("Displacement [x,y,z]: ( " + displacement[0] + "," + displacement[1] + "," + displacement[2] + " )");
	    	System.out.println("-----------------------------");
	    	wasPressed = true;
    	}
    	
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        wasPressed = false;
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
