package org.usfirst.frc.team2635.robot.commands;

import org.usfirst.frc.team2635.robot.Robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class HopperFeed extends Command {

	enum State
	{
		FEED_IN,
		FEED_SHOOTER
	}
	Timer feedTimer;
	State state = State.FEED_IN;
	double feedInTime;
	double feedShooterTime;
	
    public HopperFeed(double feedInTime, double feedShooterTime) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.hopper);
        
        feedTimer = new Timer();
        
        this.feedInTime = feedInTime;
        this.feedShooterTime = feedShooterTime;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	feedTimer.reset();
    	feedTimer.start();
    	state = State.FEED_IN;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	if(feedTimer.hasPeriodPassed(feedInTime) && state == State.FEED_IN)
    	{
    		Robot.hopper.setFeedIn(Value.kReverse);
    		Robot.hopper.setFeedShooter(Value.kForward);
    		state = State.FEED_SHOOTER;
    		feedTimer.reset();
    	}
    	else if(feedTimer.hasPeriodPassed(feedShooterTime) && state == State.FEED_SHOOTER)
    	{
    		Robot.hopper.setFeedIn(Value.kForward);
    		Robot.hopper.setFeedShooter(Value.kReverse);
    		state = State.FEED_IN;
    		feedTimer.reset();
    	}
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
