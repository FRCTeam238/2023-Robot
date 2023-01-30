package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.core238.DriverControls;
import frc.core238.DriverControls.driveType;

/**
 * OI
 */
public class OI {


	Joystick leftJoystick = RobotMap.ControlParameters.left;
	Joystick rightJoystick = RobotMap.ControlParameters.right;
	DriverControls control = new DriverControls(leftJoystick, rightJoystick, driveType.Tank);

		
}
