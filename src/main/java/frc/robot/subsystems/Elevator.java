// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.core238.PoseHelper;
import frc.robot.RobotMap;
import frc.robot.RobotMap.ElevatorParameters;
public class Elevator extends SubsystemBase {
  protected final static ElevatorFeedforward FF = new ElevatorFeedforward(ElevatorParameters.ks, ElevatorParameters.kg, ElevatorParameters.kv);
  protected final static CANSparkMax elevatorLeader = RobotMap.ElevatorParameters.elevatorLeader;
  protected final static CANSparkMax elevatorFollower = RobotMap.ElevatorParameters.elevatorFollower;
  protected Pose3d elevatorPose = new Pose3d();
  protected GenericEntry poseEntry;
  
  /** Creates a new Elevator. */
  public Elevator() {
    initSparkMax();
    poseEntry = Shuffleboard.getTab("Logging").add("ElevatorPose", PoseHelper.PoseToArray(elevatorPose)).getEntry();
  }
  
  public void initSparkMax() {
    elevatorFollower.follow(elevatorLeader);
    elevatorLeader.setSmartCurrentLimit(RobotMap.ElevatorParameters.sparkCurrentLimit);
    elevatorFollower.setSmartCurrentLimit(RobotMap.ElevatorParameters.sparkCurrentLimit);
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

  public double getPositionMeters() {
    //1.5 inches per pulley rotation divided by 42 ppr and 4:1 gear ratio
    return Units.inchesToMeters(1.5*getEncoderPosition()/(42.0*4));
  }

  public Pose3d getPose() {
    return new Pose3d(getPositionMeters()*Math.sin(Math.toRadians(35)), 0, getPositionMeters()*Math.cos(Math.toRadians(35)), new Rotation3d());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(elevatorLeader.getReverseLimitSwitch(Type.kNormallyOpen).isPressed()) {
     elevatorLeader.getEncoder().setPosition(0);
    }
    poseEntry.setDoubleArray(PoseHelper.PoseToArray(getPose()));
  }

  public void PIDdrive(TrapezoidProfile.State state) {
    double feed = FF.calculate(state.velocity);
    elevatorLeader.getPIDController().setReference(state.position, ControlType.kPosition, 0, feed);

  }

  public double getVelocity() {
    return elevatorLeader.getEncoder().getVelocity();
  }
}
