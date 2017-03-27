package org.usfirst.frc.team2635.robot.subsystems;

import org.usfirst.frc.team2635.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Hopper extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	DoubleSolenoid feedIn;
	DoubleSolenoid feedShooter;
	
	public Hopper()
	{
		feedIn = new DoubleSolenoid(RobotMap.SHOOTER_AGITATE_FORWARD, RobotMap.SHOOTER_AGITATE_REVERSE);
		feedShooter = new DoubleSolenoid(RobotMap.SHOOTER_FIRE_CONTROL_FORWARD, RobotMap.SHOOTER_FIRE_CONTROL_BACKWARDS);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    public void setFeedIn(Value value)
    {
    	feedIn.set(value);
    }
    public void setFeedShooter(Value value)
    {
    	feedShooter.set(value);
    }
}

