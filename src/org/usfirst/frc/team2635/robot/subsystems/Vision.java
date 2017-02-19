package org.usfirst.frc.team2635.robot.subsystems;

import org.usfirst.frc.team2635.robot.model.ShooterVision;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * The camera vision
 */
public class Vision extends Subsystem{

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	ShooterVision shooterVision;
	public Vision()
	{
		shooterVision = new ShooterVision();
		shooterVision.camInit();
		
	}
    public void aim()
    {
    	shooterVision.createBox();
		shooterVision.confirmBox();
		shooterVision.viewShooter();
    }
    public double getAngleToBoiler()
    {
    	return shooterVision.getAngle();
    }
    public double getDistanceToBoiler()
    {
    	return shooterVision.getDistance();
    }
	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
}

