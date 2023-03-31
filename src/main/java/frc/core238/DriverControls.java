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
	 *          <p>
	 *          Enumeration of the current type of control system we are using to
	 *          control the drivetrain <br>
	 *          this will allow us to test different modes other than tank
	 **/
	public static enum driveType {
		Tank,
		Arcade,
		Cheesy
	}

	/**
	 * {@summary} constructor for DriverControls Class
	 * 
	 * @param right
	 *                    right Driver {@link Joystick}
	 * @param left
	 *                    left Driver {@link Joystick}
	 * @param controlType
	 *                    enumeration of what control method we are using
	 */
	public DriverControls(Joystick left, Joystick right) {
		this.left = left;
		this.right = right;
	}

	// puts a cubic on the joysticks just like last year
	public double[] getTankPowers() {
		double rightJoyValue = -right.getY();
		double leftJoyValue = -left.getY();
		double modifier = RobotMap.ControlParameters.cubicModifier;
		double leftPower;
		double rightPower;

		leftPower = modifier * Math.pow(leftJoyValue, 3) + (1 - modifier) * leftJoyValue;
		rightPower = modifier * Math.pow(rightJoyValue, 3) + (1 - modifier) * rightJoyValue;
		return new double[] { leftPower, rightPower };

	}

	public double[] getArcadePowers() {
		double rightJoyValue = Math.abs(right.getX()) > 0.05 ? -right.getX() : 0;

		double leftJoyValue = Math.abs(left.getY()) > 0.1? -left.getY() : 0;
	;
		double modifier = RobotMap.ControlParameters.cubicModifier;
		double leftPower;
		double rightPower;
		leftPower = modifier * Math.pow(leftJoyValue, 3) + (1 - modifier) * leftJoyValue;
		rightPower = modifier * Math.pow(rightJoyValue, 3) + (1 - modifier) * rightJoyValue;
		return new double[] { leftPower, rightPower };

	}

	public double[] getCheesyPowers() {
		double rightJoyValue = -right.getX();
		double leftJoyValue = -left.getY();
		return new double[] {leftJoyValue, rightJoyValue};
	}

	public boolean isCheesyTurnPressed() {
		return right.getRawButton(2);
	}
}
