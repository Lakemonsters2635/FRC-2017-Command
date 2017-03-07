
package org.usfirst.frc.team2635.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team2635.robot.commands.ClimberClimb;
import org.usfirst.frc.team2635.robot.commands.DeliverGearBackwards;
import org.usfirst.frc.team2635.robot.commands.DeliverGearForward;
import org.usfirst.frc.team2635.robot.commands.DriveCameraAnglePID;
import org.usfirst.frc.team2635.robot.commands.DriveRotateMotionMagic;
import org.usfirst.frc.team2635.robot.commands.DriveRotateNavx;
import org.usfirst.frc.team2635.robot.commands.DriveRoutine;
import org.usfirst.frc.team2635.robot.commands.DriveTeleop;
//import org.usfirst.frc.team2635.robot.commands.ExampleCommand;
import org.usfirst.frc.team2635.robot.commands.LogNavxValues;
import org.usfirst.frc.team2635.robot.commands.MotionCommandGroup;
import org.usfirst.frc.team2635.robot.commands.NavxReset;
import org.usfirst.frc.team2635.robot.commands.PickupBall;
import org.usfirst.frc.team2635.robot.commands.ShooterFire;
import org.usfirst.frc.team2635.robot.commands.ShooterRevUp;
import org.usfirst.frc.team2635.robot.commands.ShooterReverseFire;
import org.usfirst.frc.team2635.robot.commands.TeleopCommand;
import org.usfirst.frc.team2635.robot.model.MotionProfileLibrary;
import org.usfirst.frc.team2635.robot.subsystems.Climber;
import org.usfirst.frc.team2635.robot.subsystems.Drive;
import org.usfirst.frc.team2635.robot.subsystems.ExampleSubsystem;
import org.usfirst.frc.team2635.robot.subsystems.GearDeliver;
import org.usfirst.frc.team2635.robot.subsystems.Pickup;
import org.usfirst.frc.team2635.robot.subsystems.Shooter;
import org.usfirst.frc.team2635.robot.subsystems.VisionSubsystem;

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

	
	Command autonomousCommand;
	Command driveCommand;
	Command logNavxCommand;
	CommandGroup teleopCommands;
	CommandGroup logAndDrive = new CommandGroup();
	MotionCommandGroup motionCommandGroup;
	
	SendableChooser<Command> chooser = new SendableChooser<>();
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");
		oi = new OI();
		drive = new Drive();
		shooter = new Shooter();
		pickup = new Pickup();
		climber = new Climber();
		deliverer = new GearDeliver();
		
		
		vision = new VisionSubsystem();
		
		teleopCommands = new TeleopCommand();
	    //motionCommandGroup = new MotionCommandGroup();
		//logAndDrive.addParallel(new LogNavxValues());
		//logAndDrive.addParallel(new DriveTeleop());
		//chooser.addDefault("Default Auto", new ExampleCommand());
		//chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", chooser);
		
		


		
		motionCommandGroup = MotionProfileLibrary.getCenterGearPlacementSequence();
		//motionCommandGroup = MotionProfileLibrary.getLeftGearPlacementSequence();
		//motionCommandGroup = MotionProfileLibrary.RotateSequence();
		

		
		
		oi.fireButton.whileHeld(new ShooterRevUp());
		oi.fireButton.whenReleased(new ShooterReverseFire());
		//oi.fireButton.whenPressed(new ShooterRevUp());
		//oi.fireButton.whenReleased(new ShooterReverseFire());
		
		oi.feedInButton.whileHeld(new PickupBall(-1.0));
		oi.feedOutButton.whileHeld(new PickupBall(1.0));
		
		oi.climbUpButton.whileHeld(new ClimberClimb());
		//oi.climbDownButton.whileHeld(new ClimberClimb(1.0));
		
		oi.deliverButton.whenPressed(new DeliverGearForward());
		oi.deliverButton.whenReleased(new DeliverGearBackwards());
		
	
		//oi.aimCameraButton.whileHeld(new DriveCameraAnglePID());//new DriveCamera(RobotMap.AIM_P, RobotMap.AIM_I, RobotMap.AIM_D));
		
		oi.navxGetAngleButton.whenReleased(new LogNavxValues());
		oi.navxResetButton.whenReleased(new NavxReset());
		
		
		
		//oi.rotateMotionMagicButton.whenPressed(new DriveRotateMotionMagic(200,90 , 36, true, true));
		//oi.motionMagicButton.whenPressed(motionCommandGroup);
		//double targetAngle = 90;
		//oi.navxRotateButton.whenPressed(new DriveRotateNavx(targetAngle) );
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		System.out.println("disabledInit");
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
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		//autonomousCommand = chooser.getSelected();
//		autonomousCommand = new DriveRoutine();

//		 try {
//			Thread.sleep(1000);
//		} catch (InterruptedException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}
		
		if (motionCommandGroup != null){
			
			motionCommandGroup.start();
		}
	
		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

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
//		if (autonomousCommand != null)
//			autonomousCommand.cancel();
//		if (teleopCommands != null)
//			teleopCommands.start();
		//logAndDrive.start();
		//Drive will run joystick automatically
		

		
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
