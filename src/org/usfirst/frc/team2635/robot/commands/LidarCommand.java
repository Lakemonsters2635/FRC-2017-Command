package org.usfirst.frc.team2635.robot.commands;

import java.util.ArrayList;

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
    	
		//Robot.drive.tankDrive(1, 1);

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.lidar.StartScanning();
    	sample = Robot.lidar.getSample();
    	ArrayList<Double> leftData = new ArrayList<Double>();
    	ArrayList<Double> rightData = new ArrayList<Double>();
    	rightData.clear();
    	leftData.clear();
    	
    	for(int i = 0;i < sample.length;i++ ) {
    		double a = (double) sample[i].angle/1000;
    		System.out.println(a + "," + sample[i].distance);
    		if(a>=70 && a<=110 && a!=90) {
    			double insideAngle = 0;
    			insideAngle = Math.abs(90-a);
    			leftData.add(Math.cos(insideAngle) * sample[i].distance);
    		} else if(a>=250 && a<=290 && a!=270) {
    			double insideAngle = 0;
    			insideAngle = Math.abs(270-a);
    			rightData.add(Math.cos(insideAngle) * sample[i].distance);
    		}
   
//    		if(a >= 160 && a <= 200) {
//    			System.out.println("Angle: " + a + " Distance: "+ sample[i].distance + " Signal Strength: " + sample[i].signalStrength);
//    			if(sample[i].distance < 100) {
//    				System.out.println("Angle: " + a + " Distance: "+ sample[i].distance + " Signal Strength: " + sample[i].signalStrength + "OH NO!");
//    				Robot.drive.tankDrive(0, 0);
//    			} else {
//    				//Robot.drive.tankDrive(1, 1);
//    			}
//    		}		
    	}
    	int leftAverage = 0;
    	for(int i = 0; i < leftData.size();i++) {
    		leftAverage += leftData.get(i);
    	}
    	leftAverage /= leftData.size();
    	
    	int rightAverage = 0;
    	for(int i = 0; i < rightData.size();i++) {
    		rightAverage += rightData.get(i);
    	}
    	rightAverage /= rightData.size();
    	System.out.println("Left Average: " + leftAverage + " Right Average: " + rightAverage);
    	if(rightAverage > leftAverage) {
    		//go right
    	} else if(leftAverage > rightAverage) {
    		//go left
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
