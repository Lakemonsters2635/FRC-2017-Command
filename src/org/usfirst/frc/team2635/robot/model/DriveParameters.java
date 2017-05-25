package org.usfirst.frc.team2635.robot.model;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

@XmlRootElement
public class DriveParameters {
	
	@XmlElement
	public double maxAcceleration;
	@XmlElement
	public double maxVelocity;
	@XmlElement
	public double leftWheelRotations;
	@XmlElement
	public double rightWheelRotations;
	
}

