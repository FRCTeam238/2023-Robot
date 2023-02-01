package frc.robot.commands;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.core238.DriverControls;
import frc.core238.DriverControls.driveType;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;

/**
 * TankDrive
 */
public class Drive extends CommandBase{

	DriverControls controls;
	DifferentialDrive diff;	
	Drivetrain drivetrain;
	driveType controlType;
	public Drive(driveType controlType) {
		drivetrain = new Drivetrain();
		controls = new DriverControls(RobotMap.ControlParameters.left,
							     RobotMap.ControlParameters.right,
							     controlType);	
		this.controlType = controlType;
		addRequirements(drivetrain);
	}
	@Override
	public void initialize() {
		// TODO Auto-generated method stub
		super.initialize();
	}

	@Override
	public void execute() {
		
		double leftOutput;
		double rightOutput;
		switch (controlType) {
			case Tank:
				leftOutput = controls.getTankPowers()[0];
				rightOutput = controls.getTankPowers()[1];
				diff.tankDrive(leftOutput, rightOutput, false);
			break;
			case Arcade:
				leftOutput = controls.getArcadePowers()[0];
				rightOutput = controls.getArcadePowers()[1];
				diff.arcadeDrive(leftOutput, rightOutput);
			break;
		}

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
