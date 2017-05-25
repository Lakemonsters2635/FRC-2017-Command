package org.usfirst.frc.team2635.robot.commands;

import org.usfirst.frc.team2635.robot.Robot;
import org.usfirst.frc.team2635.robot.RobotMap;
import org.usfirst.frc.team2635.robot.model.MotionParameters;
import org.usfirst.frc.team2635.robot.model.MotionProfileLibrary;

import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveStraightTeleop extends Command {

	double targetDistanceInches = 0.0;
    public DriveStraightTeleop() {
    	requires(Robot.drive);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    	System.out.println("*******DriveStraightTeleop initialize****************");
    	targetDistanceInches = 0.0;
    	Robot.drive.disableTeleop();
    	Robot.drive.initMotionMagic();
    	Robot.drive.setMotionMagicPIDF(
    			RobotMap.DRIVE_STRAIGHT_TELEOP_MOTION_MAGIC_P,
    			RobotMap.DRIVE_STRAIGHT_TELEOP_MOTION_MAGIC_I,
    			RobotMap.DRIVE_STRAIGHT_TELEOP_MOTION_MAGIC_D,
    			RobotMap.DRIVE_STRAIGHT_TELEOP_MOTION_MAGIC_F);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//Robot.drive.tankDrive(Robot.oi.getLeftY(), Robot.oi.getRightY());
    	double leftJoyStickValue = Robot.oi.getLeftY();
    	double MaxRPM = 450;
    	double targetRPM = (MaxRPM * leftJoyStickValue);
    	
    	double rps = targetRPM/60;
    	double rpms = rps/1000;
    	double incrementInches = (rpms * 20) * RobotMap.WHEEL_RADIUS_INCHES * Math.PI * 2;
    	targetDistanceInches += incrementInches;

    	MotionParameters driveParams = MotionProfileLibrary.getDriveParameters(RobotMap.WHEEL_RADIUS_INCHES, targetDistanceInches, MaxRPM, false);
    	Robot.drive.driveStraightMotionMagic(driveParams);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drive.enableTeleop();
       	Robot.drive.setDriveMode(TalonControlMode.PercentVbus);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.drive.enableTeleop();
       	Robot.drive.setDriveMode(TalonControlMode.PercentVbus);
    }
}
