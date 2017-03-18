package org.usfirst.frc.team2635.robot.subsystems;

import org.usfirst.frc.team2635.robot.RobotMap;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * The pickup mechanism
 */
public class Pickup extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	CANTalon pickupMotor;
	public Pickup() {
		pickupMotor = new CANTalon(RobotMap.PICKUP_BALL);
		pickupMotor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	
	}
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void feed(double magnitude) {
    	pickupMotor.set(magnitude);
    }
}

