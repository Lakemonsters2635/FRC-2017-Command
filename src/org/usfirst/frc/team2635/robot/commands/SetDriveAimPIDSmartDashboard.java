package org.usfirst.frc.team2635.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team2635.robot.Robot;
import org.usfirst.frc.team2635.robot.RobotMap;

/**
 * Set the angle pid controller to a value based on the smartdashboard
 */
public class SetDriveAimPIDSmartDashboard extends Command {

    public SetDriveAimPIDSmartDashboard() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        //requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        SmartDashboard.putNumber("Aim P", RobotMap.AIM_P);
        SmartDashboard.putNumber("Aim I", RobotMap.AIM_I);
        SmartDashboard.putNumber("Aim D", RobotMap.AIM_D);

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        double p = SmartDashboard.getNumber("Aim P", RobotMap.AIM_P);
        double i = SmartDashboard.getNumber("Aim I", RobotMap.AIM_I);
        double d = SmartDashboard.getNumber("Aim D", RobotMap.AIM_D);


        Robot.drive.setAnglePID(p, i, d);
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
