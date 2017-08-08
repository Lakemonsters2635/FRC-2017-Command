package org.usfirst.frc.team2635.robot.commands;

import org.usfirst.frc.team2635.robot.Robot;
import org.usfirst.frc.team2635.robot.RobotMap;
import org.usfirst.frc.team2635.robot.subsystems.LidarSubsystem;

import com.armabot.Sweep;
import com.armabot.SweepSample;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.TimedCommand;


/**
 *
 */
public class LidarCommand extends Command {
	
	LidarSubsystem lidar;
	SweepSample[] sample;
    public LidarCommand() {
        
        requires(Robot.lidar);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
//    	System.out.println("Lidar Start");
//    	System.load("/usr/local/frc/lib/libsweepDriver.so");
//    	sweep = new Sweep(RobotMap.SWEEP_PORT, 115200 );
//    	//System.out.println(sweep.getMotorSpeed());
//    	//sweep.reset();
//    	//sweep.setMotorSpeed(10);
//    	sweep.startScanning();
    	
		Robot.drive.tankDrive(1, 1);

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.lidar.StartScanning();
    	sample = Robot.lidar.getSample();
    	
    	for(int i = 0;i < sample.length;i++ ) {
    		double a = (double) sample[i].angle/1000;
    		
    	
    		if(a >= 160 && a <= 200) {
    			System.out.println("Angle: " + a + " Distance: "+ sample[i].distance + " Signal Strength: " + sample[i].signalStrength);
    			if(sample[i].distance < 100) {
    				System.out.println("Angle: " + a + " Distance: "+ sample[i].distance + " Signal Strength: " + sample[i].signalStrength + "OH NO!");
    				Robot.drive.tankDrive(0, 0);
    			} else {
    				Robot.drive.tankDrive(1, 1);
    			}
    		}
    		
    		
    	}
    	
    	
    	
    }

    // Called once after timeout
    protected void end() {
//    	System.out.println("Start End");
//    
//    	SweepSample[] samples = sweep.getScan();
//    	for (int i=0;i<samples.length;i++){
//    		double a = (double) samples[i].angle;
//    		System.out.println("Angle: " + a/1000 + " Distance: "+ samples[i].distance + " Signal Strength: " + samples[i].signalStrength);
//    	}
//    	System.out.println("End End");
//    	sweep.reset();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}
}
