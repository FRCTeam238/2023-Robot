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
import edu.wpi.first.wpilibj.util.Color;
import frc.core238.DriverControls.driveType;

/**
 * RobotMap
 */
public final class RobotMap {

    public static class DrivetrainParameters {
        public static final int rightDTLeaderID = 14;
        public static final int leftDTLeaderID = 0;
        public static final int rightDTFollowerID = 15;
        public static final int leftDTFollowerID = 1;
        public static final WPI_TalonFX rightDrivetrainLeader = new WPI_TalonFX(rightDTLeaderID);
        public static final WPI_TalonFX leftDrivetrainLeader = new WPI_TalonFX(leftDTLeaderID);
        public static final WPI_TalonFX leftDrivetrainFollower = new WPI_TalonFX(leftDTFollowerID);
        public static final WPI_TalonFX rightDrivetrainFollower = new WPI_TalonFX(rightDTFollowerID);
        public static final double currentLimit = 40;
        public static final double triggerThresholdCurrent = 55;
        public static final double triggerThresholdTime = 0.5;
        public static final double trackWidth = .52192;
        public static final double sensorUnitsPerRotation = 2048 * 10.86;
        public static final double wheelDiameterInches = 6.214;
        public static final double wheelCircumferenceInches = wheelDiameterInches * Math.PI;
        public static final double wheelCircumferenceMeters = Units.inchesToMeters(wheelCircumferenceInches);
        public static final double maxVoltage = 12;
        public static final double kS = 0.151;
        public static final double kV = 2.368;
        public static final double kA = 0.307;
        public static final double kP = .05;
        public static final double kD = 0;
        public static final double maxVelocity = 2.75;
        public static final double maxVelocityFF = 3;
        public static final double maxAccel = 2.25;
        public static final double maxYTolerance = 1; // meters
        public static final double maxXTolerance = 1; // meters
        public static final double maxAngle = 7.1;// in degrees
        public static final double kVAngular = 1.3288;
        public static final double kAAngular = .58306;
        public static final double kPSpin = .0055;
        public static final double kISpin = 0;
        public static final double kDSpin = 0.0007;
        public static final double angleVelocityTolerance = 5;
        public static final double minTurnValue = 0.065;
        public static final double angleTolerance = 3;

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
        public static double toleranceRotations = .7;//inchesToTicks(.5);
        public static double toleranceVelocity = .2; // in rotatation per sec

        public static final double cubeFloor = inchesToTicks(9.5);
        public static final double tippedConeFloor = 0;
        public static final double standingCone = 11.4;
        public static final double coneLevel1 = inchesToTicks(0);
        public static final double cubeLevel1 = inchesToTicks(0);
//        public static final double coneLevel2 = inchesToTicks(10.89);
        public static final double coneLevel2 = 9.8;
        public static final double cubeLevel2 = 12.9;
//        public static final double coneLevel3 = inchesToTicks(35.65);
        public static final double coneLevel3 = 24;
        public static final double cubeLevel3 = 29.5;
        public static final double doubleSubCone = 8.47;
        public static final double doubleSubConeStanding = 18.5;
        public static final double doubleSubCube = inchesToTicks(23.85);
        public static final double singleSubCone = inchesToTicks(8.13);
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
        public static double armMultiplier = 0.2;//0.125;
        public static double armThreshold = 0.05;

        // we don't use this value, we have a sendable chooser for this in robot.java,
        // this is just backup
        public static driveType controlType = driveType.Tank;

        /**
         * {@summary enum that corresponds with the angle values of the dpad}
         */
        public enum DpadDirection {
            UP(0),
            DOWN(180),
            LEFT(270),
            RIGHT(90);

            public final int value;
            DpadDirection(int i) {
                value = i;
            }
        }
    }

    public static class IntakeParameters {
        public static int intakeID = 9;
        public static int shortArmForwardChannel = 3;
        public static int shortArmBackChannel = 2;
        public static int longArmForwardChannel = 1;
        public static int longArmBackChannel = 0;
        public static int intakeSolenoidForwardChannel = 4;
        public static int intakeSolenoidBackChannel = 5;
        public static double armDelay = 1.15;
        public static double stallCurrent = 12;

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
        public static final double peakDuration = .100;
        public static final double holdSpeedCone = .2;
        public static final double holdSpeedCube = .1;
        public static final double intakeSpeedCone = .75;
        public static final double intakeSpeedCube = .75;
        public static final double outtakeSpeedCube = .5;
        public static final double outtakeSpeedCone = .3;
        public enum Gamepiece {CONE, CUBE}

        // team color yellow to represent cones
        public static Color yellow = new Color(238, 170, 0);
        // color picker'd from the CAD drawing of the cube
        public static Color purple = new Color(199, 73, 199);

    }

    public static class ArmParameters {
        public static final int armUpperLimit = 1919-25;  //In Talon native units About 2 degrees (~25 units) short of max
        public static final int armLowerLimit = -821+25;  //In Talon native units, about 2 degrees (~25 units) above min
        public static final int armOffset = 3631;      //In Talon native units, Absolute encoder value at arm horizontal
        public static TalonSRX armMotor = new TalonSRX(2);
        public static double kG = 1.24;
        public static double kV = 2.29;
        public static double kA = 0.11;
        public static double kS = 0;
        public static double kP = 1.2;
        public static double kI = 0;
        public static double kD = 0;

        public static double armCurrentLimit = 11; //?;

        public static final double cubeFloor = -64;
        public static final double tippedConeFloor = -39.8;
        public static final double standingCone = -71;
        public static final double cubeLevel1 = 82;
        public static final double cubeLevel2 = 59;
        public static final double cubeLevel3 = 40;


        public static final double coneLevel2 = 93;
        public static final double coneLevel3 = 68.5;
        public static final double coneLevel1 = 147;
        public static final double doubleSubCone = 117;
        public static final double doubleSubCube = 103.33;
        public static final double singleSubCone = 161;
        public static final double stow = 161;
        public static double maxJerk = 3000;
        public static double maxAccel = 5000;
        public static double maxVelocity = 150;
        public static double velocityTolerance = 2;
        public static double tolerance = 5;
        public static final double voltageLimit = 3;
    }

    public static class VisionParameters {

        public static Port navxPort = Port.kMXP;

    }

    public static class TrajectoryParameters {
    }

    

}
