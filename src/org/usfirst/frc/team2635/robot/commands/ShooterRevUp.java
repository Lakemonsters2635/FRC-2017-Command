package org.usfirst.frc.team2635.robot.commands;

import org.usfirst.frc.team2635.robot.Robot;
import org.usfirst.frc.team2635.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Spin up the flywheel and agitate.
 */
public class ShooterRevUp extends CommandGroup {

    public ShooterRevUp() {
    	//Change this to sequential when a method to figure out if it's finished is implemented
    	requires(Robot.shooter);
    	//addParallel(new ShooterFire());
    	
    	
    	//addParallel(new ShooterAgitate(RobotMap.SHOOTER_AGITATE_TIME));
    	
    	addSequential(new ShooterSetAgitator(Value.kForward));
    	addSequential(new WaitCommand(RobotMap.SHOOTER_AGITATE_TIME));
    	addSequential(new ShooterSetAgitator(Value.kReverse));
    	addSequential(new WaitCommand(RobotMap.SHOOTER_AGITATE_TIME)); 
    	addSequential(new ShooterFire(Value.kForward));
    	addSequential(new WaitCommand(RobotMap.SHOOTER_AGITATE_TIME)); 
    	addSequential(new ShooterFire(Value.kReverse));
    	
        //addSequential(new ShooterSpinFlywheel(-1.0));
    	
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    }
}
