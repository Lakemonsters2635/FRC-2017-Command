package org.usfirst.frc.team2635.robot.commands;

import org.usfirst.frc.team2635.robot.Robot;

import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2635.robot.subsystems.Drive;

/**
 * Drive using joysticks and PercentVBus
 */
public class DriveTeleop extends Command {

    public DriveTeleop() {
        requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        Robot.drive.setDriveMode(TalonControlMode.PercentVbus);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        /*System.out.printf("Joystick left: %f, Joystick right: %f \n", Robot.oi.getLeftY(), Robot.oi.getRightY());
        Robot.drive.tankDriveVoltage(Robot.oi.getLeftY(), Robot.oi.getRightY());
        Robot.drive.scootch(Robot.oi.getLeftThrottle());*/
        Robot.drive.tankDrive(Robot.oi.getLeftY(), Robot.oi.getRightY(), Robot.oi.getLeftThrottle());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.drive.tankDriveVoltage(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        Robot.drive.tankDriveVoltage(0, 0);
    }
}
