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

public static class DrivetrainParameters {
    public static int rightDTLeaderID = 14;
    public static int leftDTLeaderID = 0;
    public static int rightDTFollowerID = 15;
    public static int leftDTFollowerID = 1;
    public static TalonFX rightDrivetrainLeader = new TalonFX(rightDTLeaderID);
    public static TalonFX leftDrivetrainLeader = new TalonFX(leftDTLeaderID);
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
    
		//TODO: Do we want to test out other driver control methods like arcade/cheesy?
		//we don't use this value, we have a sendable chooser for this in robot.java, this is just backup
		public static driveType controlType = driveType.Tank;

	}
}
