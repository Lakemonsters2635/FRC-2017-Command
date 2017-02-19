package org.usfirst.frc.team2635.robot.subsystems;

import org.usfirst.frc.team2635.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * The gear delivery mechanism
 */
public class GearDeliver extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	DoubleSolenoid deliverer;
    public GearDeliver()
    {
    	deliverer = new DoubleSolenoid(RobotMap.DELIVER_GEAR_FORWARD, RobotMap.DELIVER_GEAR_BACKWARDS);
    }
	public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	
    }
    public void setForward()
    {
    	deliverer.set(Value.kForward);
    }
    public void setBackwards()
    {
    	deliverer.set(Value.kReverse);
    }
}

