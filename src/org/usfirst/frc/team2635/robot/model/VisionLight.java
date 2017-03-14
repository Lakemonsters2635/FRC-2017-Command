package org.usfirst.frc.team2635.robot.model;

import edu.wpi.first.wpilibj.Solenoid;


public class VisionLight {
	Solenoid vlight;
	
	public VisionLight(int channel) {
		vlight = new Solenoid(channel);
		
	}
	public void lightOn() {
		vlight.set(true);
	}
	
	public void lightOff() {
		vlight.set(false);
	}
}
