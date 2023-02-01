package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import frc.core238.DriverControls.driveType;

/**
 * RobotMap
 */
public final class RobotMap {

	public static class Drivetrain {
		public static int rightDTLeaderID;
		public static int leftDTLeaderID;
		public static int rightDTFollowerID;
		public static int leftDTFollowerID;
		public static TalonFX rightDrivetrainLeader = new TalonFX(rightDTLeaderID);
		public static TalonFX leftDrivetrainLeader = new TalonFX(leftDTLeaderID);
		public static TalonFX leftDrivetrainFollower = new TalonFX(leftDTFollowerID);
		public static TalonFX rightDrivetrainFollower = new TalonFX(rightDTFollowerID);

	}
	public static class Elevator {
		public static int elevatorFollowerID = 13; 
		public static int elevatorLeaderID = 12;
		public static CANSparkMax elevatorLeader = new CANSparkMax(elevatorLeaderID, MotorType.kBrushless);
		public static CANSparkMax elevatorFollower = new CANSparkMax(elevatorFollowerID, MotorType.kBrushless);
		//TODO: initialize these variables w/ real #'s 
		public static int sparkCurrentLimit;
		public static int softLimitForward;
		public static int softLimitBackward;
		}

	public static class ControlParameters {
		public static int leftJoyID = 0;
		public static int rightJoyID = 1;
		public static int xboxID = 2;
		public static double cubicModifier = 0.2;
		public static Joystick left = new Joystick(leftJoyID);
		public static Joystick right = new Joystick(rightJoyID);
		// TODO: Do we want to test out other driver control methods like arcade/cheesy?
		public static driveType controlType = driveType.Tank;

	}
}
