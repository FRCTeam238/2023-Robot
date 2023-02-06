// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.RobotMap.ElevatorParameters;

public class Elevator extends SubsystemBase {
  protected final static ElevatorFeedforward FF = new ElevatorFeedforward(ElevatorParameters.ks, ElevatorParameters.kg, ElevatorParameters.kv);
  protected final static CANSparkMax elevatorLeader = RobotMap.ElevatorParameters.elevatorLeader;
  protected final static CANSparkMax elevatorFollower = RobotMap.ElevatorParameters.elevatorFollower;
  
  /** Creates a new Elevator. */
  public Elevator() {
    initSparkMax();
  }
  
  public void initSparkMax() {
    elevatorFollower.follow(elevatorLeader);
    elevatorLeader.setSmartCurrentLimit(RobotMap.ElevatorParameters.sparkCurrentLimit);
    elevatorLeader.setSoftLimit(SoftLimitDirection.kForward, RobotMap.ElevatorParameters.softLimitForward);
    elevatorLeader.setSoftLimit(SoftLimitDirection.kReverse, RobotMap.ElevatorParameters.softLimitBackward);
    elevatorLeader.getForwardLimitSwitch(Type.kNormallyOpen).enableLimitSwitch(true);
    elevatorLeader.getReverseLimitSwitch(Type.kNormallyOpen).enableLimitSwitch(true);
    elevatorLeader.getPIDController().setP(ElevatorParameters.kp);
    elevatorLeader.getPIDController().setP(ElevatorParameters.ki);
    elevatorLeader.getPIDController().setP(ElevatorParameters.kd);
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

  public void PIDdrive(TrapezoidProfile.State state) {
    double feed = FF.calculate(state.velocity);
    elevatorLeader.getPIDController().setReference(state.position, ControlType.kPosition, 0, feed);

  }

  public double getVelocity() {
    return elevatorLeader.getEncoder().getVelocity();
  }
}
