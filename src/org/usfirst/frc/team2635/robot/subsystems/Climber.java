package org.usfirst.frc.team2635.robot.subsystems;

import org.usfirst.frc.team2635.robot.Robot;
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
	
	public static double totalPowerLimit = 298;
	
    public Climber()
    {
    	climb1 = new CANTalon(RobotMap.ROPE_CLIMBER_1);
    	climb1.changeControlMode(TalonControlMode.Voltage);
    	
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
    	
    	
    	double climb1Amps = climb1.getOutputCurrent();
    	double climb2Amps = climb2.getOutputCurrent();
    	System.out.println("---------------------------------------");
    	System.out.println("Your amperage is: " + climb1Amps);
    	
    	double climb1Voltage = climb1.getOutputVoltage();
    	double climb2Voltage = climb2.getOutputVoltage();
    	System.out.println("Your voltage is: " + climb1Voltage);
    	
    	double climb1Watts = Math.abs(climb1Amps * climb1Voltage);
    	double climb2Watts = Math.abs(climb2Amps * climb2Voltage);
    	System.out.println("Your wattage is: " + climb1Watts);
    	double totalWatts = climb1Watts + climb2Watts;
    	
    	double limitVoltage = Math.signum(magnitude)*totalPowerLimit/(climb1Amps + climb2Amps);
    	
    	if(totalWatts > totalPowerLimit){
    		System.out.println("Limiting Voltage to: " + limitVoltage);
    		climb1.set(limitVoltage);
    	}
    	else{
    		System.out.println("Setting Voltage to: " + magnitude);
    		climb1.set(magnitude);
    	}
    
    	
    }
}	

