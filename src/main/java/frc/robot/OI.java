package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.core238.DriverControls;
import frc.core238.DriverControls.driveType;
import frc.robot.commands.CloseIntakeCommand;
import frc.robot.commands.Drive;
import frc.robot.commands.ElevatorManualCommand;

/**
 * OI
 */
public class OI {


	Joystick leftJoystick = RobotMap.ControlParameters.left;
	Joystick rightJoystick = RobotMap.ControlParameters.right;
	XboxController operatorController = RobotMap.ControlParameters.operatorController;
	DriverControls control = new DriverControls(leftJoystick, rightJoystick);

	ElevatorManualCommand elevatorManualCommand;
	Drive driveCommand;

	public OI(driveType controlType) {
		
		driveCommand = new Drive();
		elevatorManualCommand = new ElevatorManualCommand();
		Robot.drivetrain.setDefaultCommand(driveCommand);	
		Robot.elevator.setDefaultCommand(elevatorManualCommand);

		JoystickButton closeIntake = new JoystickButton(operatorController, 0);
		closeIntake.onTrue(new CloseIntakeCommand());

	}
	
		
}
