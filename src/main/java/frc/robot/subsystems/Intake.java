// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.core238.PoseHelper;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {
  
  TalonSRX intakeMotor;
  GenericEntry armPoseEntry;
  GenericEntry armTextEntry;
  Pose3d armPose = new Pose3d();

  DoubleArrayLogEntry logPose;
  BooleanLogEntry logShort;
  BooleanLogEntry logLong;
  BooleanLogEntry logIntake;
  StringLogEntry logCommand;
  DoubleLogEntry logSpeed;
  
  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = RobotMap.IntakeParameters.intakeMotor;
    intakeMotor.configContinuousCurrentLimit(RobotMap.IntakeParameters.continuousCurrent);
    intakeMotor.configPeakCurrentLimit(RobotMap.IntakeParameters.peakCurrent);
    intakeMotor.configPeakCurrentDuration(RobotMap.IntakeParameters.peakDuration);
    intakeMotor.enableCurrentLimit(true);
    intakeMotor.setNeutralMode(NeutralMode.Brake);

    logShort = new BooleanLogEntry(DataLogManager.getLog(), "Intake:Short");
    logLong = new BooleanLogEntry(DataLogManager.getLog(), "Intake:Long");
    logPose = new DoubleArrayLogEntry(DataLogManager.getLog(), "Intake:Pose");
    logIntake = new BooleanLogEntry(DataLogManager.getLog(), "Intake:Grip");
    logCommand = new StringLogEntry(DataLogManager.getLog(), "Intake:Command");
    logSpeed = new DoubleLogEntry(DataLogManager.getLog(), "Intake:Speed");

    retractLong();
    retractShort();
    closeIntake();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    armPose = Robot.elevator.getPose();
    if (getShort())
    {
      armPose = armPose.plus(new Transform3d(new Translation3d(-.08,0,.065), new Rotation3d(0, Math.toRadians(19), 0)));
    }
    if (getLong())
    {
      armPose = armPose.plus(new Transform3d(new Translation3d(-.15,0,.32), new Rotation3d(0, Math.toRadians(67.4), 0)));
    }
    logPose.append(PoseHelper.PoseToArray(armPose));
  }

  public void putCommandString(Command command) {
    logCommand.append(command.getName());
  }

  public void runIntake(double intakeSpeed) {
    intakeMotor.set(ControlMode.PercentOutput, intakeSpeed);
    if (intakeMotor.getSupplyCurrent() > RobotMap.IntakeParameters.autoCloseCurrent) {
      closeIntake();
      if (!DriverStation.isAutonomous()) {
        RobotMap.ControlParameters.operatorController.setRumble(RumbleType.kBothRumble, 1);
      }
    } else {
      RobotMap.ControlParameters.operatorController.setRumble(RumbleType.kBothRumble, 0);
    }
    logSpeed.append(intakeSpeed);
  }

  public Value getIntakePosition() {
    return RobotMap.IntakeParameters.intakeSolenoid.get();
  }

  public void openIntake() {
    RobotMap.IntakeParameters.intakeSolenoid.set(Value.kReverse);
    logIntake.append(false);
  }

  public void closeIntake() {
    RobotMap.IntakeParameters.intakeSolenoid.set(Value.kForward);
    logIntake.append(true);
  }

  public void extendLong() {
    RobotMap.IntakeParameters.longArm.set(Value.kForward);
    logLong.append(true);
  }

  public void retractLong() {
    RobotMap.IntakeParameters.longArm.set(Value.kReverse);
    logLong.append(false);
  }

  public void extendShort() {
    RobotMap.IntakeParameters.shortArm.set(Value.kForward);
    logShort.append(true);
  }

  public void retractShort() {
    RobotMap.IntakeParameters.shortArm.set(Value.kReverse);
    logShort.append(false);
  }

  public boolean getShort() {
    if (RobotMap.IntakeParameters.shortArm.get() == Value.kForward) {
      return true;
    } else {
      return false;
    }
  }

  public boolean getLong() {
    if (RobotMap.IntakeParameters.longArm.get() == Value.kForward) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isEitherExtended() {
    return getLong() || getShort();
  }

}
