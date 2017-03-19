package org.usfirst.frc.team2635.robot.subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team2635.robot.RobotMap;

/**
 * The climbing mechanism
 */
public class Climber extends Subsystem {

    public static double totalPowerLimit = 400;
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    CANTalon climb1;
    CANTalon climb2;

    public Climber() {
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

    public void climb(double magnitude) {
        double climb1Amps = climb1.getOutputCurrent();
        double climb2Amps = climb2.getOutputCurrent();
        System.out.println("---------------------------------------");
        System.out.println("climb1Amps is: " + climb1Amps);
        System.out.println("climb2Amps is: " + climb2Amps);

        double climb1Voltage = climb1.getOutputVoltage();
        double climb2Voltage = climb2.getOutputVoltage();
        System.out.println("climb1Voltage is: " + climb1Voltage);
        System.out.println("climb2Voltage is: " + climb2Voltage);

        double climb1Watts = Math.abs(climb1Amps * climb1Voltage);
        double climb2Watts = Math.abs(climb2Amps * climb2Voltage);
        System.out.println("climb1Watts is: " + climb1Watts);
        System.out.println("climb2Watts is: " + climb2Watts);

        double totalWatts = climb1Watts + climb2Watts;
        System.out.println("totalWatts is: " + totalWatts);
        double limitVoltage = Math.signum(magnitude) * totalPowerLimit / (climb1Amps + climb2Amps);

        if (totalWatts > totalPowerLimit) {
            System.out.println("Limiting Voltage to: " + limitVoltage);
            climb1.set(limitVoltage);
        } else {
            System.out.println("Setting Voltage to: " + magnitude);
            climb1.set(magnitude);
        }
    }
}	

