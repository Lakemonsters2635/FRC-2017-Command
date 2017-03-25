package org.usfirst.frc.team2635.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2635.robot.Robot;
import org.usfirst.frc.team2635.robot.subsystems.Drive;

/**
 * Created by Ethan on 3/22/2017.
 * Receives a parameter for when it is called upon a particular button press and sends it to setTankDriveMode
 * to change current teleop drive operation as well as conduct proper initialization for said drive mode.
 */
public class TankDriveSwitchMode extends Command {
    private Drive.TankDriveMode tankDriveMode;

    public TankDriveSwitchMode(Drive.TankDriveMode tankDriveMode) {
        this.tankDriveMode = tankDriveMode;
    }

    protected void initialize() {
        if (tankDriveMode == Drive.TankDriveMode.MOTION_MAGIC || tankDriveMode == Drive.TankDriveMode.SCOOTCH) {
            Robot.drive.initMotionMagic();
        } else if (tankDriveMode == Drive.TankDriveMode.VOLTAGE) {
            Robot.drive.initVoltageDrive();
        }
//        System.out.println("Current tank drive mode: " + Robot.drive.getTankDriveMode());
    }

    protected void execute() {
        Robot.drive.setTankDriveMode(tankDriveMode);
    }

    protected boolean isFinished() {
        return false;
    }
}
