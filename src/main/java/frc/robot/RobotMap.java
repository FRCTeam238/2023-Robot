package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.pathplanner.lib.PathConstraints;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SPI.Port;
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
    public static WPI_TalonFX rightDrivetrainLeader = new WPI_TalonFX(rightDTLeaderID);
    public static WPI_TalonFX leftDrivetrainLeader = new WPI_TalonFX(leftDTLeaderID);
    public static WPI_TalonFX leftDrivetrainFollower = new WPI_TalonFX(leftDTFollowerID);
	public static WPI_TalonFX rightDrivetrainFollower = new WPI_TalonFX(rightDTFollowerID);
	public static double currentLimit = 40;
	public static double triggerThresholdCurrent = 55;
	public static double triggerThresholdTime = 0.5;
	public static double trackWidth;
	public static double wheelCircumferenceMeters;
	public static double sensorUnitsPerRotation = 2048 * 10.86;
	public static double wheelDiameterInches = 6.18;
	public static double wheelCircumferenceInches = wheelDiameterInches * Math.PI;
	public static double maxVoltage = 12; 
	public static double kS; 
	public static double kV; 
	public static double kA;
	public static double maxVelocity = 2;
	public static double maxAccel = 2; 
	public static double maxYTolerance = 0.05; // meters
	public static double maxXTolerance = 0.05; // meters
	public static double maxAngle = 1;// in degrees 

	}

	public static class ElevatorParameters {
		public static final double MaxVel = 0;
        public static final double MaxAccel = 0;
        public static int elevatorFollowerID = 13; 
		public static int elevatorLeaderID = 12;
		public static CANSparkMax elevatorLeader = new CANSparkMax(elevatorLeaderID, MotorType.kBrushless);
		public static CANSparkMax elevatorFollower = new CANSparkMax(elevatorFollowerID, MotorType.kBrushless);
		//TODO: initialize these variables w/ real #'s 
		public static int sparkCurrentLimit;
		public static int softLimitForward;
		public static int softLimitBackward;
		public static double kv;
		public static double kg;
		public static double ks;
		public static double ka;
		public static double kp;
		public static double ki;
		public static double kd;
        public static double midCubeHeight;
        public static double midConeHeight;
		public static double floorHeight;
		public static double topHeight;
		}

	public static class ControlParameters {
		public static int leftJoyID = 1;
		public static int rightJoyID = 2;
		public static int xboxID = 0;
		public static double cubicModifier = 0.2;
		public static Joystick left = new Joystick(leftJoyID);
		public static Joystick right = new Joystick(rightJoyID);
		public static XboxController operatorController = new XboxController(xboxID);
		public static double elevatorThreshold = 0.2;
		public static double elevatorMultiplier = 0.25;
		
    
		//we don't use this value, we have a sendable chooser for this in robot.java, this is just backup
		public static driveType controlType = driveType.Tank;

	}
	
	public static class IntakeParameters {
		public static int intakeID = 9;
		public static int shortArmForwardChannel = 5;
		public static int shortArmBackChannel = 4;
		public static int longArmForwardChannel = 7;
		public static int longArmBackChannel = 6;
		public static int intakeSolenoidForwardChannel = 1;
		public static int intakeSolenoidBackChannel = 0;

		public static double intakeSpeed = 0.2;
		public static double intakeEjectSpeed = 0.2;

		public static VictorSPX intakeMotor = new VictorSPX(intakeID);

		public static DoubleSolenoid shortArm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, shortArmForwardChannel, shortArmBackChannel);
		public static DoubleSolenoid longArm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, longArmForwardChannel, longArmBackChannel);
		public static DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, intakeSolenoidForwardChannel, intakeSolenoidBackChannel);
	}

	public static class VisionParameters {

        public static Port navxPort;

	}
}
