package org.usfirst.frc.team2635.robot.commands;

import org.usfirst.frc.team2635.robot.Robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Set the agitator to a certain magnitude
 */
public class ShooterAgitate extends Command {
	Value magnitude;
	Value currentMagnitude;
	//boolean isPaused;
//	int countMax;
//	int count;
	double agitateTime;
	Timer agitaiteTimer = new Timer();
    public ShooterAgitate(double agitateTime) {
        // Use requires() here to declare subsystem dependencies
    	//requires(Robot.shooter);
    	//this.magnitude = Value.kForward;
    	this.currentMagnitude = Value.kForward;
    	this.agitateTime = agitateTime;
    	//    	this.countMax = countMax;
//    	count = countMax;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
//    	count = countMax;
    	agitaiteTimer.reset();
    	agitaiteTimer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
//    	count++;
//    	if(count > countMax) {
//    	count = 0;
//    	if(currentMagnitude == Value.kForward)
//    	{
//    		currentMagnitude = Value.kReverse;
//    	}
//    	else
//    	{
//    		currentMagnitude = Value.kForward;
//    	}
//	    
//    		if(!isPaused) {
//	    		currentMagnitude = Value.kReverse;
//	    		isPaused = true;
//	    		if(magnitude == Value.kForward)
//	        	{
//	        		magnitude = Value.kReverse;
//	        	}
//	        	else
//	        	{
//	        		magnitude = Value.kForward;
//	        	}
//	    		System.out.println("isPaused");
//	    	} else {
//	    		currentMagnitude = magnitude;
//	    		isPaused = false;
//	    	}
//    	}
    	if(agitaiteTimer.hasPeriodPassed(agitateTime))
    	{
        	if(currentMagnitude == Value.kForward)
        	{
        		currentMagnitude = Value.kReverse;
        	}
        	else
        	{
        		currentMagnitude = Value.kForward;
        	}
        	agitaiteTimer.reset();
    	}
    	Robot.shooter.setAgitator(currentMagnitude);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.shooter.setAgitator(Value.kReverse);
    	agitaiteTimer.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.shooter.setAgitator(Value.kReverse);
    	agitaiteTimer.stop();

    }
}
