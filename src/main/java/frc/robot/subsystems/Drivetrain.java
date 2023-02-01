// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Drivetrain extends SubsystemBase {
  public static final TalonFX leftControllerDrive = RobotMap.DrivetrainParameters.leftDrivetrainLeader;
  public static final TalonFX rightControllerDrive = RobotMap.DrivetrainParameters.rightDrivetrainLeader;

  public static final TalonFX leftFollower = RobotMap.DrivetrainParameters.leftDrivetrainFollower;
  public static final TalonFX rightFollower = RobotMap.DrivetrainParameters.rightDrivetrainFollower;

	

  /** Creates a new Drivetrain. */
  public Drivetrain() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void resetOdometry(Pose2d pose) {

  }

  public void driveByPercentOutput(double left, double right) {
	rightControllerDrive.set(ControlMode.PercentOutput, right);
	leftControllerDrive.set(ControlMode.PercentOutput, left);
	
  }	  

  public void driveStraight(double left, double right) {
	double avg = (left + right)/2;
	rightControllerDrive.set(ControlMode.PercentOutput, avg);
	leftControllerDrive.set(ControlMode.PercentOutput, avg);
  }
}
