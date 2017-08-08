package org.usfirst.frc.team2635.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team2635.robot.RobotMap;

import com.armabot.Sweep;
import com.armabot.SweepSample;
/**
 *
 */
public class LidarSubsystem extends Subsystem {

    Sweep sweep;
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
	public LidarSubsystem() {
		System.load("/usr/local/frc/lib/libsweepDriver.so");
		sweep = new Sweep(RobotMap.SWEEP_PORT, 115200);
		
	}
	
	public SweepSample[] getSample() {
		return sweep.getScan();
	}
	
	public void StartScanning(){
		sweep.startScanning();
	}
	public void Reset(){
		sweep.reset();
	}
	public int GetMotorSpeed(){
		return sweep.getMotorSpeed();
	}
    public void SetMotorSpeed(int speed){
    	sweep.setMotorSpeed(speed);
    }
    public void stopScanning(){
    	sweep.stopScanning();
    }
}

