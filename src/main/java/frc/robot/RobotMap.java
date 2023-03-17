package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
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
        public static double trackWidth = .52192;
        public static double sensorUnitsPerRotation = 2048 * 10.86;
        public static double wheelDiameterInches = 6.214;
        public static double wheelCircumferenceInches = wheelDiameterInches * Math.PI;
        public static double wheelCircumferenceMeters = Units.inchesToMeters(wheelCircumferenceInches);
        public static double maxVoltage = 12;
        public static double kS = 0.151;
        public static double kV = 2.368;
        public static double kA = 0.307;
        public static double kP = .05;
        public static double kD = 0;
        public static double maxVelocity = 2.75;
        public static double maxVelocityFF = 3;
        public static double maxAccel = 2.25;
        public static double maxYTolerance = 1; // meters
        public static double maxXTolerance = 1; // meters
        public static double maxAngle = 4;// in degrees
        public static final double kVAngular = 1.3288;
        public static final double kAAngular = .58306;
        public static final double kPSpin = .0055;
        public static final double kISpin = 0;
        public static final double kDSpin = 0.0007;
        public static final double angleVelocityTolerance = 5;
        public static final double minTurnValue = 0.065;
        public static double angleTolerance = 1;

    }

    public static class ElevatorParameters {
        protected final static int cpr = 1;
        protected final static double gearing = 4; // 4:1 gear ratio
        protected final static double inchesPerRev = 1.5 * Math.PI; // 1.5" diameter pulley

        public static double inchesToTicks(double inches) {
            return (inches / inchesPerRev) * gearing * cpr;
        }

        public static final double MaxVel = inchesToTicks(60); // Starting very slow. Real max is ~ 8 fps = 8*12
        public static final double MaxAccel = inchesToTicks(100); // ~1/3 second to accell to this slow max V
        public static final double holdPercent = 0.03;
        public static int elevatorFollowerID = 13;
        public static int elevatorLeaderID = 12;
        public static CANSparkMax elevatorLeader = new CANSparkMax(elevatorLeaderID, MotorType.kBrushless);
        public static CANSparkMax elevatorFollower = new CANSparkMax(elevatorFollowerID, MotorType.kBrushless);

        public static int sparkCurrentLimit = 30;
        public static double softLimitForward = inchesToTicks(30); // Actual max travel = 36
        public static double softLimitBackward = inchesToTicks(.25);
        public static double kv = .12; // theortical
        public static double kg = .4;
        public static double ks = .185;
        public static double ka = .002; // theortical
        public static double kp = .15;
        public static double ki;
        public static double kd;
        public static double midCubeHeight = inchesToTicks(21.25); // initial guess based on CAD
        public static double midConeHeight = 29.22;// inchesToTicks(31.175); //initial guess based on CAD
        public static double floorHeight = inchesToTicks(.25); // drive all the way down to soft limit
        public static double topHeight = inchesToTicks(35); // initial guess based on CAD
        public static double toleranceRotations = inchesToTicks(.5);
        public static double toleranceVelocity = .2; // in rotatation per sec

    }

    public static class ControlParameters {
        public static int leftJoyID = 1;
        public static int rightJoyID = 2;
        public static int xboxID = 0;
        public static double cubicModifier = 0.2;
        public static Joystick left = new Joystick(leftJoyID);
        public static Joystick right = new Joystick(rightJoyID);
        public static XboxController operatorController = new XboxController(xboxID);
        public static double elevatorThreshold = 0.1;
        public static double elevatorMultiplier = 0.125;

        // we don't use this value, we have a sendable chooser for this in robot.java,
        // this is just backup
        public static driveType controlType = driveType.Tank;

    }

    public static class IntakeParameters {
        public static int intakeID = 9;
        public static int shortArmForwardChannel = 3;
        public static int shortArmBackChannel = 2;
        public static int longArmForwardChannel = 1;
        public static int longArmBackChannel = 0;
        public static int intakeSolenoidForwardChannel = 4;
        public static int intakeSolenoidBackChannel = 5;
        public static double armDelay = 1.25;
        public static int autoCloseCurrent = 12;

        public static double intakeSpeed = 0.5;
        public static double intakeEjectSpeed = .5;

        public static TalonSRX intakeMotor = new TalonSRX(intakeID);

        public static DoubleSolenoid shortArm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, shortArmForwardChannel,
                shortArmBackChannel);
        public static DoubleSolenoid longArm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, longArmForwardChannel,
                longArmBackChannel);
        public static DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
                intakeSolenoidForwardChannel, intakeSolenoidBackChannel);

        public static final int continuousCurrent = 10;
        public static final int peakCurrent = 20;
        public static final int peakDuration = 60;
    }

    public static class VisionParameters {

        public static Port navxPort = Port.kMXP;

    }

    public static class TrajectoryParameters {
    }
}
