package org.usfirst.frc.team2635.robot.model;

public class SensorParameters {

	public Double AngleToTarget;
	public Double DistanceToTarget;  //Represents the Distance Detected by Vision
	public double DistanceAdjustment; //Represents an adjustment (if needed) to stop short of distance. Default is 0.
	
	public SensorParameters(Double angle, Double distance) {
		this.AngleToTarget = angle;
		this.DistanceToTarget = distance;
	}	
}
