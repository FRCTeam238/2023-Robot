package frc.core238;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotMap;

/**
 * {@summary Class for holding the joystick related methods.}
 * Throw joysticks in, get values out. thats it
 */
public class DriverControls {
	public Joystick left;
	public Joystick right;
	driveType controlType;

	/**
	 * @summary 
	 * <p> Enumeration of the current type of control system we are using to control the drivetrain <br>
	 * this will allow us to test different modes other than tank
	 **/
	public static enum driveType {
		Tank,
		Arcade,
		Cheesy //TODO: do we even want to try to make cheesy right now?
	}

	/**
	 * {@summary} constructor for DriverControls Class
	 * @param right
	 * right Driver {@link Joystick} 
	 * @param left
	 * left Driver {@link Joystick}
	 * @param controlType
	 * enumeration of what control method we are using
	 * */
	public DriverControls(Joystick left, Joystick right, driveType controlType) {
		this.left = left;
		this.right = right;
		this.controlType = controlType;
	}



	//puts a cubic on the joysticks just like last year
	public double[] getPowers() {
		double rightJoyValue = right.getY();
		double leftJoyValue = left.getY();
		double modifier = RobotMap.ControlParameters.cubicModifier;
		double leftPower;
		double rightPower;
		if (controlType == driveType.Tank) {
			leftPower = modifier * Math.pow(leftJoyValue, 3) + (1 - modifier) * leftJoyValue;
			rightPower = modifier * Math.pow(rightJoyValue, 3) + (1 - modifier) * rightJoyValue;
			return new double[]{leftPower, rightPower};
		} else {

			return new double[]{0, 0};
		}

		
	}
}
