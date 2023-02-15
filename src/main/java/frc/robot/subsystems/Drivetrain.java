// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotMap;

public class Drivetrain extends SubsystemBase {
  public static final WPI_TalonFX leftControllerDrive = RobotMap.DrivetrainParameters.leftDrivetrainLeader;
  public static final WPI_TalonFX rightControllerDrive = RobotMap.DrivetrainParameters.rightDrivetrainLeader;

  public static final WPI_TalonFX leftFollower = RobotMap.DrivetrainParameters.leftDrivetrainFollower;
  public static final WPI_TalonFX rightFollower = RobotMap.DrivetrainParameters.rightDrivetrainFollower;
  private static final int kTimeoutMs = 30;
  private static final int kPIDLoopIdx = 0;
  private static final double kP = 0;
  private static final double kI = 0;
  private static final double kD = 0;
  private static final double kF = 0;
  public static DifferentialDrive diff = new DifferentialDrive(leftControllerDrive, rightControllerDrive);
  public static DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(RobotMap.DrivetrainParameters.trackWidth);
  public static DifferentialDriveOdometry differentialDriveOdometry;

  public static SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(RobotMap.DrivetrainParameters.kS, RobotMap.DrivetrainParameters.kV, RobotMap.DrivetrainParameters.kA);

  protected Field2d robotPose;
  
  Pose2d currentPose;
  AHRS navx;

  DifferentialDrivetrainSim m_driveSim;
  TalonFXSimCollection m_leftDriveSim = leftControllerDrive.getSimCollection();
  TalonFXSimCollection m_rightDriveSim = rightControllerDrive.getSimCollection();

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    navx = new AHRS(RobotMap.VisionParameters.navxPort);
    differentialDriveOdometry = new DifferentialDriveOdometry(navx.getRotation2d(), getLeftEncoderTicks(), getRightEncoderTicks());
    initTalons();
    
    robotPose = new Field2d();
    Shuffleboard.getTab("Logging").add("RobotPose", robotPose);

    m_driveSim = new DifferentialDrivetrainSim(
      // Create a linear system from our identification gains.
      LinearSystemId.identifyDrivetrainSystem(RobotMap.DrivetrainParameters.kVLinear, RobotMap.DrivetrainParameters.kALinear, RobotMap.DrivetrainParameters.kVAngular, RobotMap.DrivetrainParameters.kAAngular),
      DCMotor.getFalcon500(2),       // 2 Falcon motors on each side of the drivetrain.
      10.86,                    // 10.86:1 gearing reduction.
      RobotMap.DrivetrainParameters.trackWidth,                  
      Units.inchesToMeters(RobotMap.DrivetrainParameters.wheelDiameterInches/2),
      null);
  }

  public void initTalons() {
    rightControllerDrive.setInverted(true);
    rightFollower.setInverted(true);
    leftFollower.follow(leftControllerDrive);
    rightFollower.follow(rightControllerDrive);

    rightControllerDrive.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);

    leftControllerDrive.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Brake);
    rightControllerDrive.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, RobotMap.DrivetrainParameters.currentLimit, RobotMap.DrivetrainParameters.triggerThresholdCurrent, 0.5));
    leftControllerDrive.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, RobotMap.DrivetrainParameters.currentLimit, RobotMap.DrivetrainParameters.triggerThresholdCurrent, 0.5));
    rightFollower.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, RobotMap.DrivetrainParameters.currentLimit, RobotMap.DrivetrainParameters.triggerThresholdCurrent, 0.5));
    leftFollower.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, RobotMap.DrivetrainParameters.currentLimit, RobotMap.DrivetrainParameters.triggerThresholdCurrent, 0.5));

    //Set the encoder status frame to update every 1ms to minimize latency and jitter
    initPID(leftControllerDrive);
    initPID(rightControllerDrive);
    
  }

  public void initPID(TalonFX talon) {
       /* Config the peak and nominal outputs ([-1, 1] represents [-100, 100]%) */
       talon.configNominalOutputForward(0, kTimeoutMs);
       talon.configNominalOutputReverse(0, kTimeoutMs);
       talon.configPeakOutputForward(1, kTimeoutMs);
       talon.configPeakOutputReverse(-1, kTimeoutMs);
   
       /**
        * Config the allowable closed-loop error, Closed-Loop output will be
        * neutral within this range.
        */
       talon.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);
   
       /* Config closed loop gains for Primary closed loop (Current) */
       talon.config_kP(kPIDLoopIdx, kP, kTimeoutMs);
       talon.config_kI(kPIDLoopIdx, kI, kTimeoutMs);
       talon.config_kD(kPIDLoopIdx, kD, kTimeoutMs);
       talon.config_kF(kPIDLoopIdx, kF, kTimeoutMs);
       //talon.config_IntegralZone(kPIDLoopIdx, kIzone, kTimeoutMs);
       
       // Config encoders and set them to update every 1ms to minimize latency/jitter
       talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
       talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1, 0);
   
       //talon.configClosedloopRamp(rampRate, kTimeoutMs);
   
       // Ensure motor output and encoder velocity are proportional to each other
       // If they become inverted, set these to true
       talon.setSensorPhase(true);
 
       talon.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_5Ms);
       talon.configVelocityMeasurementWindow(2);
   
       talon.setSelectedSensorPosition(0);
     }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    differentialDriveOdometry.update(navx.getRotation2d(), stepsToMeters(getLeftEncoderTicks()), stepsToMeters(getRightEncoderTicks()));
    currentPose = differentialDriveOdometry.getPoseMeters();
    robotPose.setRobotPose(currentPose);
  }

  public void resetOdometry(Pose2d pose) {
    currentPose = pose;
  }

  public Pose2d getCurrentPose() {
    return differentialDriveOdometry.getPoseMeters();
  }

  public double getRightEncoderTicks() {
    return rightControllerDrive.getSelectedSensorPosition();
  }


  public double getLeftEncoderTicks() {
    return leftControllerDrive.getSelectedSensorPosition();
  }

  public void driveByVelocityOutput(double left, double right) {
    rightControllerDrive.set(ControlMode.Velocity, right, DemandType.ArbitraryFeedForward, feedForward.calculate(right) / RobotMap.DrivetrainParameters.maxVoltage);
    leftControllerDrive.set(ControlMode.Velocity, left, DemandType.ArbitraryFeedForward, feedForward.calculate(left) / RobotMap.DrivetrainParameters.maxVoltage);
  }



  public void tankDrive(double left, double right) {
    diff.tankDrive(left, right, false);
  }

  public void arcadeDrive(double left, double right) {
    diff.arcadeDrive(left, right, false);
  }

  public void cheesyDrive(double xSpeed, double zRotation, boolean turnInPlace) {
    diff.curvatureDrive(xSpeed, zRotation, turnInPlace);
  }


  
  public void driveStraight(double left, double right) {
    double avg = (left + right) / 2.0;
    rightControllerDrive.set(ControlMode.PercentOutput, avg);
    leftControllerDrive.set(ControlMode.PercentOutput, avg);
  }



  public static double stepsToMeters(double steps) {
    return (RobotMap.DrivetrainParameters.wheelCircumferenceMeters / RobotMap.DrivetrainParameters.sensorUnitsPerRotation) * steps;
  }

  public static double metersToSteps(double meters) {
    return (meters / RobotMap.DrivetrainParameters.wheelCircumferenceMeters) * RobotMap.DrivetrainParameters.sensorUnitsPerRotation;
  }

  public static double stepsPerDecisecToMetersToSec(double stepsPerDecisec) {
    return stepsToMeters(stepsPerDecisec * 10.0);
  }

  public static double metersPerSecToStepsPerDecisec(double metersPerSec) {
    return metersToSteps(metersPerSec) / 10.0;
  }

  public static double insToRevs(double inches) {
   return inches / RobotMap.DrivetrainParameters.wheelCircumferenceInches;
  }

  public static double insToSteps(double inches) {
    return insToRevs(inches) * RobotMap.DrivetrainParameters.sensorUnitsPerRotation;
  }

  public static double insPerSecToStepsToDecisec(double inchesPerSecond) {
    return insToSteps(inchesPerSecond) * 0.1;
  }

  @Override
  public void simulationPeriodic()
  {
    m_leftDriveSim.setBusVoltage(12);
    m_rightDriveSim.setBusVoltage(12);
    m_driveSim.setInputs(-m_leftDriveSim.getMotorOutputLeadVoltage(),
                         m_rightDriveSim.getMotorOutputLeadVoltage());

    // Advance the model by 20 ms. Note that if you are running this
    // subsystem in a separate thread or have changed the nominal timestep
    // of TimedRobot, this value needs to match it.
    m_driveSim.update(0.02);

    // Update all of our sensors.
    leftControllerDrive.getSimCollection().setIntegratedSensorRawPosition((int) metersToSteps(m_driveSim.getLeftPositionMeters()));
    leftControllerDrive.getSimCollection().setIntegratedSensorVelocity((int) metersPerSecToStepsPerDecisec(m_driveSim.getLeftVelocityMetersPerSecond()));
    rightControllerDrive.getSimCollection().setIntegratedSensorRawPosition(-(int) metersToSteps(m_driveSim.getRightPositionMeters()));
    rightControllerDrive.getSimCollection().setIntegratedSensorVelocity(-(int) metersPerSecToStepsPerDecisec(m_driveSim.getRightVelocityMetersPerSecond()));    
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    angle.set(-m_driveSim.getHeading().getDegrees());
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_driveSim.getCurrentDrawAmps()));
  }

}
