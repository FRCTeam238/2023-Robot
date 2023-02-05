package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.core238.DriverControls;
import frc.core238.DriverControls.driveType;
import frc.robot.commands.Drive;

/**
 * OI
 */
public class OI {


	Joystick leftJoystick = RobotMap.ControlParameters.left;
	Joystick rightJoystick = RobotMap.ControlParameters.right;
	DriverControls control = new DriverControls(leftJoystick, rightJoystick);

	Drive driveCommand;
	public OI(driveType controlType) {
		
		driveCommand = new Drive();
		Robot.drivetrain.setDefaultCommand(driveCommand);	

	}

		
}
