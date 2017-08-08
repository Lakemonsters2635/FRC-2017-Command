package org.usfirst.frc.team2635.robot.commands;

import org.usfirst.frc.team2635.robot.RobotMap;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RotationTest extends Command {

    public RotationTest() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	CANTalon rightFront;
    	CANTalon leftFront;
    	CANTalon rightBack;
    	CANTalon leftBack;
    	rightFront = new CANTalon(RobotMap.DRIVE_RIGHT_FRONT);
    	leftFront = new CANTalon(RobotMap.DRIVE_LEFT_FRONT);
		rightBack = new CANTalon(RobotMap.DRIVE_RIGHT_BACK);
		leftBack = new CANTalon(RobotMap.DRIVE_LEFT_BACK);
		
		double rightFrontPosition = rightFront.getPosition();
		double leftFrontPosition = leftFront.getPosition();
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
   
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
