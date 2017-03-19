package org.usfirst.frc.team2635.robot.commands;

import java.time.LocalDateTime;

import org.usfirst.frc.team2635.robot.Robot;
import org.usfirst.frc.team2635.robot.RobotMap;
import org.usfirst.frc.team2635.robot.model.MotionParameters;
import org.usfirst.frc.team2635.robot.model.MotionProfileLibrary;
import org.usfirst.frc.team2635.robot.model.UltrasonicParameters;

import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveStraightMotionMagic extends Command {
    public double driveDistance;
    public boolean reverse;
    double rpm;
    public int cycleCtr;
    UltrasonicParameters ultraSonicParams;

    MotionParameters driveParams;

    public DriveStraightMotionMagic(double rpm, double driveDistance, boolean reverse) {
        requires(Robot.drive);
        this.rpm = rpm;
        this.driveDistance = driveDistance;
        this.reverse = reverse;
    }

    public DriveStraightMotionMagic(double rpm, UltrasonicParameters ultrasonicParams) {
        this.ultraSonicParams = ultrasonicParams;
        this.rpm = rpm;
        this.driveDistance = 0;
        this.reverse = false;

    }

    // Called just before this Command runs the first time
    protected void initialize() {
        System.out.println("DrivesStraightMotionMagic initialized at " + LocalDateTime.now());

        if (ultraSonicParams != null && ultraSonicParams.rightInches != null) {

            //targetAngle = visionParams.AngleToTarget;
            System.out.println("Get Drive Params by UltraSonicParams.rightInches:" + ultraSonicParams.rightInches);
            driveParams = MotionProfileLibrary.getDriveParameters(RobotMap.WHEEL_RADIUS_INCHES, ultraSonicParams.rightInches, rpm, reverse);

        } else {
            System.out.println("Get Drive Params by fixed distance:" + driveDistance);
            driveParams = MotionProfileLibrary.getDriveParameters(RobotMap.WHEEL_RADIUS_INCHES, driveDistance, rpm, reverse);
        }

        Robot.drive.initMotionMagic();
        Robot.drive.setMotionMagicPIDF(
                RobotMap.DRIVE_STRAIGHT_MOTION_MAGIC_P,
                RobotMap.DRIVE_STRAIGHT_MOTION_MAGIC_I,
                RobotMap.DRIVE_STRAIGHT_MOTION_MAGIC_D,
                RobotMap.DRIVE_STRAIGHT_MOTION_MAGIC_F);

        //Robot.drive.driveStraightMotionMagic(driveParams);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Robot.drive.driveStraightMotionMagic(driveParams);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        boolean done = Robot.drive.motionMagicDone(driveParams, Robot.drive.DRIVE_ERROR_TOLERANCE);
        if (done) {
            System.out.println("DriveStraightMotionMagic is done");

            if (ultraSonicParams != null) {
                ultraSonicParams.leftInches = null;
                ultraSonicParams.rightInches = null;
            }

        }
        return done;
        //return false;
    }

    // Called once after isFinished returns true
    protected void end() {
        System.out.println("DriveStraightMotionMagic end");
        Robot.drive.initMotionMagic();
        Robot.drive.setDriveMode(TalonControlMode.PercentVbus);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        //System.out.println("DriveStraightMotionMagic interrupted");
        Robot.drive.setDriveMode(TalonControlMode.PercentVbus);
    }
}
