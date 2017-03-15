package org.usfirst.frc.team2635.robot.commands;

import org.usfirst.frc.team2635.robot.RobotMap;

public class CommonCaluclations {
	public int calcEncCounts(double distance) {
		return (int) (distance * RobotMap.DISTANCE_PER_WHEEL_REVOLUTION);
	}
}
