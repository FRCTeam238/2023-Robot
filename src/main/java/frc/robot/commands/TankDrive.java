package frc.robot.commands;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.core238.DriverControls;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;

/**
 * TankDrive
 */
public class TankDrive extends CommandBase{

	DriverControls controls;
	DifferentialDrive diff;	
	Drivetrain drivetrain;
	public TankDrive() {
		drivetrain = new Drivetrain();
		controls = new DriverControls(RobotMap.ControlParameters.left,
							     RobotMap.ControlParameters.right,
							     RobotMap.ControlParameters.controlType);	
		addRequirements(drivetrain);
	}
	@Override
	public void initialize() {
		// TODO Auto-generated method stub
		super.initialize();
	}

	@Override
	public void execute() {
		double leftOutput = controls.getPowers()[0];
		double rightOutput = controls.getPowers()[1];

		drivetrain.driveByPercentOutput(leftOutput, rightOutput);
	}

	@Override
	public void end(boolean interrupted) {
		// TODO Auto-generated method stub
		super.end(interrupted);
	}

	@Override
	public boolean isFinished() {
		// TODO Auto-generated method stub
		return super.isFinished();
	}
}
