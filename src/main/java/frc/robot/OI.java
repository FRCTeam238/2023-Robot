package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.core238.DriverControls;
import frc.core238.DriverControls.driveType;
import frc.robot.commands.Drive;
import frc.robot.commands.ElevatorManualCommand;

/**
 * OI
 */
public class OI {


	Joystick leftJoystick = RobotMap.ControlParameters.left;
	Joystick rightJoystick = RobotMap.ControlParameters.right;
	DriverControls control = new DriverControls(leftJoystick, rightJoystick);

	ElevatorManualCommand elevatorManualCommand;
	Drive driveCommand;
	public OI(driveType controlType) {
		
		driveCommand = new Drive();
		elevatorManualCommand = new ElevatorManualCommand();
		Robot.drivetrain.setDefaultCommand(driveCommand);	
		Robot.elevator.setDefaultCommand(elevatorManualCommand);

	}
		
}
