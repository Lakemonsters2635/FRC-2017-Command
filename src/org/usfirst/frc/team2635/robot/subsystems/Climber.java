package org.usfirst.frc.team2635.robot.subsystems;

import org.usfirst.frc.team2635.robot.RobotMap;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * The climbing mechanism
 */
public class Climber extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	CANTalon climb1;
	CANTalon climb2;
    public Climber()
    {
    	climb1 = new CANTalon(RobotMap.ROPE_CLIMBER_1);
    	
    	climb2 = new CANTalon(RobotMap.ROPE_CLIMBER_2);
    	climb2.changeControlMode(TalonControlMode.Follower);
    	climb2.set(climb1.getDeviceID());
    }
	public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    public void climb(double magnitude)
    {
    	climb1.set(magnitude);	
    }
}	

