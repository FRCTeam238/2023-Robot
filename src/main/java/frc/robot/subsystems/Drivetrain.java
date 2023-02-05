// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Drivetrain extends SubsystemBase {
  public static final WPI_TalonFX leftControllerDrive = RobotMap.DrivetrainParameters.leftDrivetrainLeader;
  public static final WPI_TalonFX rightControllerDrive = RobotMap.DrivetrainParameters.rightDrivetrainLeader;

  public static final WPI_TalonFX leftFollower = RobotMap.DrivetrainParameters.leftDrivetrainFollower;
  public static final WPI_TalonFX rightFollower = RobotMap.DrivetrainParameters.rightDrivetrainFollower;
  public static DifferentialDrive diff = new DifferentialDrive(leftControllerDrive, rightControllerDrive);

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    initTalons();
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

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void resetOdometry(Pose2d pose) {

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
    double avg = (left + right) / 2;
    rightControllerDrive.set(ControlMode.PercentOutput, avg);
    leftControllerDrive.set(ControlMode.PercentOutput, avg);
  }
}
