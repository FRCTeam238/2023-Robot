// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  public Arm() {
  
  }

  public void moveArmPercent (double armPercent, double armVelocity) {
    RobotMap.ArmParameters.armMotor.set(ControlMode.PercentOutput, armPercent);
  }

  public void moveArmVelocity() {
  }
  
  public void initControls() {
    // set current limit
    // setup encoder
    // set PID values
    // set soft limits
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
