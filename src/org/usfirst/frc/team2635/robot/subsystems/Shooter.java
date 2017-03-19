package org.usfirst.frc.team2635.robot.subsystems;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team2635.robot.RobotMap;

/**
 * The shooter mechanism
 */
public class Shooter extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    CANTalon flywheel;
    CANTalon agitator;
    DoubleSolenoid fireControl;

    public Shooter() {
        flywheel = new CANTalon(RobotMap.SHOOTER_BALL_FLYWHEEL);
        flywheel.changeControlMode(CANTalon.TalonControlMode.PercentVbus);

        agitator = new CANTalon(RobotMap.SHOOTER_AGITATOR);
        agitator.changeControlMode(CANTalon.TalonControlMode.PercentVbus);

        fireControl = new DoubleSolenoid(RobotMap.SHOOTER_FIRE_CONTROL_FORWARD, RobotMap.SHOOTER_FIRE_CONTROL_BACKWARDS);

    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

    public void setFlywheel(double magnitude) {
        flywheel.set(magnitude);
    }

    public void setAgitator(double magnitude) {
        agitator.set(magnitude);
    }

    public void fireControlForward() {
        fireControl.set(Value.kForward);
    }

    public void fireControlReverse() {
        fireControl.set(Value.kReverse);
    }

}

