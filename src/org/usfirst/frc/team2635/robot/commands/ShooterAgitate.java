package org.usfirst.frc.team2635.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2635.robot.Robot;

/**
 * Set the agitator to a certain magnitude
 */
public class ShooterAgitate extends Command {
    double magnitude;
    double currentMagnitude;
    boolean isPaused;
    int countMax;
    int count;

    public ShooterAgitate(double magnitude, int countMax) {
        // Use requires() here to declare subsystem dependencies
        //requires(Robot.shooter);
        this.magnitude = magnitude;
        this.currentMagnitude = magnitude;
        this.countMax = countMax;
        count = countMax;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        count = countMax;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        count++;
        if (count > countMax) {
            count = 0;
            currentMagnitude = -currentMagnitude;

            if (!isPaused) {
                currentMagnitude = 0;
                isPaused = true;
                magnitude = -magnitude;
                System.out.println("isPaused");
            } else {
                currentMagnitude = magnitude;
                isPaused = false;
            }
        }
        Robot.shooter.setAgitator(currentMagnitude);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.shooter.setAgitator(0.0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        Robot.shooter.setAgitator(0.0);
    }
}
