package org.usfirst.frc.team2635.robot.subsystems;

import org.usfirst.frc.team2635.robot.RobotMap;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Drive extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	CANTalon rightFront;
	CANTalon rightBack;
	CANTalon leftFront;
	CANTalon leftBack;
	RobotDrive drive;
	
	public Drive()
	{
		rightFront = new CANTalon(RobotMap.DRIVE_RIGHT_FRONT);
		
		rightBack = new CANTalon(RobotMap.DRIVE_RIGHT_BACK);
		rightBack.changeControlMode(TalonControlMode.Follower);
		rightBack.set(rightFront.getDeviceID());
		
		leftFront = new CANTalon(RobotMap.DRIVE_LEFT_FRONT);
		
		leftBack = new CANTalon(RobotMap.DRIVE_LEFT_BACK);
		leftBack.changeControlMode(TalonControlMode.Follower);
		leftBack.set(leftFront.getDeviceID());
		
		drive = new RobotDrive(leftFront, rightFront);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void tankDrive(double left, double right)
    {
    	drive.tankDrive(left, right);
    }
}

