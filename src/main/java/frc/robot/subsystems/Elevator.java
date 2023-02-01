// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Elevator extends SubsystemBase {
  
  protected final static CANSparkMax elevatorLeader = RobotMap.Elevator.elevatorLeader;
  protected final static CANSparkMax elevatorFollower = RobotMap.Elevator.elevatorFollower;
  
  /** Creates a new Elevator. */
  public Elevator() {
    initSparkMax()
  }
  
  public void initSparkMax() {
    elevatorFollower.follow(elevatorLeader);
    elevatorLeader.setSmartCurrentLimit(RobotMap.Elevator.sparkCurrentLimit);
    elevatorLeader.setSoftLimit(SoftLimitDirection.kForward, RobotMap.Elevator.softLimitForward);
    elevatorLeader.setSoftLimit(SoftLimitDirection.kReverse, RobotMap.Elevator.softLimitBackward);
    elevatorLeader.getForwardLimitSwitch(Type.kNormallyOpen).enableLimitSwitch(true);
    elevatorLeader.getReverseLimitSwitch(Type.kNormallyOpen).enableLimitSwitch(true);
  }

  public void moveByPercentOutput(double percent) {
    elevatorLeader.set(percent);
  }

  public double getEncoderPosition() {
    return elevatorLeader.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(elevatorLeader.getReverseLimitSwitch(Type.kNormallyOpen).get()) {
     elevatorLeader.getEncoder().setPosition(0);
    
    }

    
  }
}
