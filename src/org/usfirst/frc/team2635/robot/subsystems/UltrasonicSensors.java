package org.usfirst.frc.team2635.robot.subsystems;


import org.usfirst.frc.team2635.robot.RobotMap;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.command.Subsystem;
/**
 *
 */


public class UltrasonicSensors extends Subsystem {

	AnalogInput leftSensor;
	AnalogInput rightSensor;
	static final double SCALE_FACTOR = 1000.0/(4.9 * 2.54);
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	public UltrasonicSensors() {
		leftSensor = new AnalogInput(RobotMap.LEFT_ULTRASONIC_CHANNEL);
		
		rightSensor =  new AnalogInput(RobotMap.RIGHT_ULTRASONIC_CHANNEL);
	}
	
	public double getLeftDistanceInches() {
		double voltage = leftSensor.getVoltage();
		return voltage * SCALE_FACTOR;
		
	}
	
	public double getRightDistanceInches() {
		double voltage = rightSensor.getVoltage();
		return voltage * SCALE_FACTOR;
	}
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

