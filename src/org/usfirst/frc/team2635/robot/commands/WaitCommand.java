package org.usfirst.frc.team2635.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class WaitCommand extends Command {

    Timer timer;
    double delaySec;

    public WaitCommand(double delaySec) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        timer = new Timer();
        this.delaySec = delaySec;
        //timer.delay(seconds);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        timer.reset();
        timer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {

        return timer.hasPeriodPassed(delaySec);

    }

    // Called once after isFinished returns true
    protected void end() {
        timer.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        timer.stop();
        timer.reset();
    }
}
