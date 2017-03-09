package org.usfirst.frc.team2635.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class LightSubsystem extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
Solenoid vlight;
	
	public LightSubsystem(int channel){
		vlight = new Solenoid(channel);
		
	}
	public void lightOn(){
		vlight.set(true);
	}
	
	public void lightOff(){
		vlight.set(false);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

