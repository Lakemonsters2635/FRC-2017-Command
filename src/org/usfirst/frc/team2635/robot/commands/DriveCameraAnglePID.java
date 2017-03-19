package org.usfirst.frc.team2635.robot.commands;

import org.usfirst.frc.team2635.robot.Robot;
import org.usfirst.frc.team2635.robot.model.VisionLight;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Drive using the navx based on the input from the camera.
 */
public class DriveCameraAnglePID extends Command {
    //degrees
    VisionLight light;

    public DriveCameraAnglePID() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);

        //FHE TODO. UNCOMMENT THE FOLLOWING LINE
        //requires(Robot.drive);
        requires(Robot.vision);
        light = new VisionLight(7);

    }

    // Called just before this Command runs the first time
    protected void initialize() {
        Robot.drive.enableAnglePID();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        light.lightOn();
        Robot.vision.aim();
        Double angle = Robot.vision.getAngleToBoiler();
        Double distance = Robot.vision.getDistanceToBoiler();

        System.out.println("angle:" + angle);
        System.out.println("distance:" + distance);
        if (angle != null) {
            System.out.println("angle NOT NULL:" + angle);
            Robot.drive.driveAnglePID(angle);
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        light.lightOff();
        return Robot.drive.angleOnTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.drive.disableAnglePID();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        Robot.drive.disableAnglePID();
    }

}
