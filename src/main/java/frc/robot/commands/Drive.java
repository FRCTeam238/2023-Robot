package frc.robot.commands;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.core238.DriverControls;
import frc.core238.DriverControls.driveType;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;

/**
 * TankDrive
 */
public class Drive extends CommandBase {

	SendableChooser<driveType> controlType = new SendableChooser<>();
	DriverControls controls;
	DifferentialDrive diff;
	Drivetrain drivetrain;

	public Drive() {

		drivetrain = Robot.drivetrain;
		controls = new DriverControls(RobotMap.ControlParameters.left,
				RobotMap.ControlParameters.right);

		addRequirements(drivetrain);
		controlType.setDefaultOption("Tank", driveType.Tank);
    	controlType.addOption("Arcade", driveType.Arcade);
		controlType.addOption("Cheesy", driveType.Cheesy);
		SmartDashboard.putData(controlType);
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
		switch (controlType.getSelected()) {
			case Tank:
				leftOutput = controls.getTankPowers()[0];
				rightOutput = controls.getTankPowers()[1];
				drivetrain.tankDrive(leftOutput, rightOutput);
				break;
			case Arcade:
				leftOutput = controls.getArcadePowers()[0];
				rightOutput = controls.getArcadePowers()[1];
				drivetrain.arcadeDrive(leftOutput, rightOutput);
				break;
			case Cheesy:
				leftOutput = controls.getCheesyPowers()[0];
				rightOutput = controls.getCheesyPowers()[1];
				drivetrain.cheesyDrive(leftOutput, rightOutput, controls.isCheesyTurnPressed());
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
