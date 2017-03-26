package org.usfirst.frc.team2635.robot.model;

public class SensorParameters {

	public Double AngleToTarget;
	public Double DistanceToTarget;  //Represents the Distance Detected by Vision
	public Double SensorFailedDriveDistance; //The Distance to use if Sensor Fails
	public double DistanceAdjustment; //Represents an adjustment (if needed) to stop short of distance. Default is 0.
	public boolean TargetAcquired = false;
	
	public SensorParameters(Double angle, Double distance) {
		this.AngleToTarget = angle;
		this.DistanceToTarget = distance;
	}	
}
