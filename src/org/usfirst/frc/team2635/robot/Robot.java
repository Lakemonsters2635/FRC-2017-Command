package org.usfirst.frc.team2635.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team2635.robot.commands.*;
import org.usfirst.frc.team2635.robot.model.MotionProfileLibrary;
import org.usfirst.frc.team2635.robot.subsystems.*;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

    //public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
    public static Drive drive;
    public static Shooter shooter;
    public static Pickup pickup;
    public static Climber climber;
    public static GearDeliver deliverer;
    public static VisionSubsystem vision;
    public static OI oi;
    public static UltrasonicSensors ultrasonic;
    public static LightSubsystem light;

    Command autonomousCommand;
    Command driveCommand;
    Command logNavxCommand;
    CommandGroup teleopCommands;
    MotionCommandGroup motionCommandGroup;

    MotionCommandGroup centerGear;
    MotionCommandGroup leftGear;
    MotionCommandGroup leftGearSimple;
    MotionCommandGroup rightGear;
    MotionCommandGroup visionTest;

    MotionCommandGroup doNothingCmd;
    MotionCommandGroup rotateTest;

    SendableChooser<Command> chooser = new SendableChooser<>();

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        System.out.println("robotInit");

        final boolean ENABLE_TANK_DRIVE_WITH_ENCODERS_FIRST = false;

        oi = new OI();
        drive = new Drive(ENABLE_TANK_DRIVE_WITH_ENCODERS_FIRST);
        shooter = new Shooter();
        pickup = new Pickup();
        climber = new Climber();
        deliverer = new GearDeliver();

        ultrasonic = new UltrasonicSensors();
        vision = new VisionSubsystem();
        light = new LightSubsystem(RobotMap.VISION_LIGHT_CHANNEL);
        teleopCommands = new TeleopCommand();

        initializeChooser();

        oi.fireButton.whileHeld(new ShooterRevUp());
        oi.fireButton.whenReleased(new ShooterReverseFire());
        //oi.fireButton.whenPressed(new ShooterRevUp());
        //oi.fireButton.whenReleased(new ShooterReverseFire());

        oi.feedInButton.whileHeld(new PickupBall(-1.0));
        oi.feedOutButton.whileHeld(new PickupBall(1.0));

        oi.deliverButton.whenPressed(new DeliverGearForward(RobotMap.GEAR_DELIVERY_TIMEOUT));
        oi.deliverButton.whenReleased(new DeliverGearBackwards(RobotMap.GEAR_DELIVERY_TIMEOUT));

        oi.climbUpButton.whileHeld(new ClimberClimb());

        oi.aimCameraButton.whileHeld(MotionProfileLibrary.visionTestSequence());

        //oi.navxGetAngleButton.whenReleased(new LogNavxValues());
        //oi.navxResetButton.whenReleased(new NavxReset());

        //oi.rotateMotionMagicButton.whenPressed(new DriveRotateMotionMagic(200,90 , 36, true, true));
        //oi.motionMagicButton.whenPressed(motionCommandGroup);
        //double targetAngle = 90;
        //oi.navxRotateButton.whenPressed(new DriveRotateNavx(targetAngle) );

    }

    public void initializeChooser() {
        doNothingCmd = MotionProfileLibrary.doNothing();
        centerGear = MotionProfileLibrary.getCenterGearPlacementSequence();
        leftGear = MotionProfileLibrary.getLeftGearPlacementSequence();
        leftGearSimple = MotionProfileLibrary.getSimpleLeftGearPlacementSequence();
        rightGear = MotionProfileLibrary.getRightGearPlacementSequence();
        visionTest = MotionProfileLibrary.visionTestSequence();
        rotateTest = MotionProfileLibrary.rotateTestSequence();

        chooser.initTable(null);
        chooser.addDefault("Do Nothing", doNothingCmd);
        chooser.addObject("Center Gear", centerGear);
        chooser.addObject("Left Gear", leftGear);
        chooser.addObject("Left Gear Simple", leftGearSimple);
        chooser.addObject("Right Gear", rightGear);
        chooser.addObject("Vision Test", visionTest);
        chooser.addObject("Rotation Test", rotateTest);

        SmartDashboard.putData("Autonomous mode", chooser);
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     * You can use it to reset any subsystem information you want to clear when
     * the robot is disabled.
     */
    @Override
    public void disabledInit() {
        System.out.println("disabledInit");
        if (motionCommandGroup != null && motionCommandGroup.isRunning()) {
            motionCommandGroup.cancel();
        }
    }

    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString code to get the auto name from the text box below the Gyro
     * <p>
     * You can add additional auto modes by adding additional commands to the
     * chooser code above (like the commented example) or additional comparisons
     * to the switch structure below with additional strings & commands.
     */
    @Override
    public void autonomousInit() {
        if (motionCommandGroup != null && motionCommandGroup.isRunning()) {
            motionCommandGroup.cancel();
        }

        initializeChooser();

        if (drive.teleopIsRunning()) {
            drive.disableTeleop();
        }

		/*
         * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */
        System.out.println("-------------------------------Started Autonomous-------------------------");
        motionCommandGroup = (MotionCommandGroup) chooser.getSelected();
        motionCommandGroup.start();

        // schedule the autonomous command (example)
//		if (autonomousCommand != null)
//			autonomousCommand.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {

//		while (isInitialized)
//		{
//			isInitialized = Initialize();
//		}

        Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (motionCommandGroup != null)
            motionCommandGroup.cancel();

        //drive.enableTeleop();
//		if (teleopCommands != null)
//			teleopCommands.start();

        //logAndDrive.start();
        //Drive will run joystick automatically

        //drive.enableTeleop();


        System.out.println("teleopInit");


    }

    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This function is called periodically during test mode
     */
    @Override
    public void testPeriodic() {
        LiveWindow.run();
    }
}
