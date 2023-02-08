// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {
  
  VictorSPX intakeMotor;
  
  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = RobotMap.IntakeParameters.intakeMotor;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void runIntake(double intakeSpeed) {
    intakeMotor.set(ControlMode.PercentOutput, intakeSpeed);
  }

  public void openIntake() {
    RobotMap.IntakeParameters.intakeSolenoid.set(Value.kReverse);
  }

  public void closeIntake() {
    RobotMap.IntakeParameters.intakeSolenoid.set(Value.kForward);
  }

  public void extendLong() {
    RobotMap.IntakeParameters.longArm.set(Value.kForward);
  }

  public void retractLong() {
    RobotMap.IntakeParameters.longArm.set(Value.kReverse);
  }

  public void extendShort() {
    RobotMap.IntakeParameters.shortArm.set(Value.kForward);
  }

  public void retractShort() {
    RobotMap.IntakeParameters.shortArm.set(Value.kReverse);

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
