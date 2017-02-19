package org.usfirst.frc.team2635.robot.model;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Navx hardware
 */
public class Navx  {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	AHRS navx;
	double accumulator = 0.0;
	Double previous = null;
	double unwrapTolerance = 180;
	public Navx()
	{
		navx = new AHRS(SPI.Port.kMXP);
	}
	/**
	 * Get the angle
	 * @return The angle from -inf to inf
	 */
    public double getAngle()
    {
    	return navx.getAngle();
    }
    /**
     * Get the angle accounting for angle jumps
     * @return The angle from -inf - inf
     */
//    public double getUnwrappedAngle()
//    {
//    	if(previous == null)
//    	{
//    		previous = navx.getAngle();
//    		return accumulator;
//    	}
//    	double current = navx.getAngle();
//    	double delta = previous-current;
//    	System.out.println("delta: " + delta);
//    	//Rotating right
//    	if(delta > unwrapTolerance)
//    	{
//    		accumulator -= delta + unwrapTolerance * 2;
//    	}
//    	//Rotating left
//    	else if(delta < -unwrapTolerance)
//    	{
//    		accumulator += delta - unwrapTolerance * 2;
//    	}
//    	else
//    	{
//    		accumulator += delta;
//    	}
//    	previous = current;
//    	return accumulator;
//    }
}

