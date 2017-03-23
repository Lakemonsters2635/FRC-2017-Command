package org.usfirst.frc.team2635.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

//import org.usfirst.frc.team2635.robot.commands.ExampleCommand;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    public final Joystick rightJoystick = new Joystick(RobotMap.JOYSTICK_RIGHT);
    public final Joystick leftJoystick = new Joystick(RobotMap.JOYSTICK_LEFT);

    public final Button revUpButton = new JoystickButton(leftJoystick, RobotMap.BUTTON_REV_UP);
    public final Button fireButton = new JoystickButton(rightJoystick, RobotMap.BUTTON_FIRE);
    public final Button feedInButton = new JoystickButton(rightJoystick, RobotMap.BUTTON_FEED_IN);
    public final Button feedOutButton = new JoystickButton(rightJoystick, RobotMap.BUTTON_FEED_OUT);
    public final Button climbUpButton = new JoystickButton(leftJoystick, RobotMap.BUTTON_CLIMB_UP);
    //public final Button climbDownButton = new JoystickButton(leftJoystick, RobotMap.BUTTON_CLIMB_DOWN);
    public final Button deliverButton = new JoystickButton(rightJoystick, RobotMap.BUTTON_DELIVER);
    public final Button aimCameraButton = new JoystickButton(rightJoystick, RobotMap.BUTTON_AIM_CAMERA);

    // teleop drive mode buttons
    public final Button motionMagicButton = new JoystickButton(rightJoystick, RobotMap.BUTTON_MOTION_MAGIC);
    public final Button voltageDriveButton = new JoystickButton(rightJoystick, RobotMap.BUTTON_VOLTAGE_DRIVE);
    public final Button scootchDriveButton = new JoystickButton(rightJoystick, RobotMap.BUTTON_SCOOTCH);

    public final Button navxRotateButton = new JoystickButton(leftJoystick, RobotMap.BUTTON_NAVX_ROTATE);
    public final Button navxGetAngleButton = new JoystickButton(rightJoystick, RobotMap.BUTTON_NAVX_GET_ANGLE);
    public final Button navxResetButton = new JoystickButton(rightJoystick, RobotMap.BUTTON_NAVX_RESET);

    public final double getLeftY() {
        return -leftJoystick.getRawAxis(RobotMap.LEFT_DRIVE_Y);
    }

    public final double getRightY() {
        return -rightJoystick.getRawAxis(RobotMap.RIGHT_DRIVE_Y);
    }

    public final double getRightZ() {
        return rightJoystick.getRawAxis(RobotMap.RIGHT_DRIVE_Z);
    }

    public final double getLeftZ() {
        return leftJoystick.getRawAxis(RobotMap.LEFT_DRIVE_Z);
    }

    public final double getLeftThrottle() {
        return leftJoystick.getThrottle();
    }

//	
    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a
    //// joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);

    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.

    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:

    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());

    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());

    // Start the command when the button is released and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());
}
