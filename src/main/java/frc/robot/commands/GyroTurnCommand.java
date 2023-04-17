// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.core238.autonomous.AutonomousModeAnnotation;
import frc.robot.Robot;
import frc.robot.RobotMap;

@AutonomousModeAnnotation(parameterNames = { "angle", "maxSpeed" })
public class GyroTurnCommand extends CommandBase {
  /** Creates a new GyroTurnCommand. */
  double angle;
  double maxSpeed;
  PIDController controller;
  boolean debug = false;

  DoubleLogEntry logSetpoint;
  DoubleLogEntry logOutput;
  DoubleLogEntry logCurrent;
  DoubleLogEntry logMax;

  /**
   *
   * @param angle in degrees
   * @param maxSpeed
   */
  public GyroTurnCommand(double angle, double maxSpeed) {
    this.angle = angle;
    this.maxSpeed = maxSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.drivetrain);
    controller = new PIDController(RobotMap.DrivetrainParameters.kPSpin, RobotMap.DrivetrainParameters.kISpin,
        RobotMap.DrivetrainParameters.kDSpin);
    controller.enableContinuousInput(-180, 180);
    controller.setTolerance(RobotMap.DrivetrainParameters.angleTolerance, RobotMap.DrivetrainParameters.angleVelocityTolerance);

    logSetpoint = new DoubleLogEntry(DataLogManager.getLog(), "Drivetrain:/GyroTurn/Setpoint");
    logOutput = new DoubleLogEntry(DataLogManager.getLog(), "Drivetrain:/GyroTurn/Output");
    logCurrent = new DoubleLogEntry(DataLogManager.getLog(), "Drivetrain:/GyroTurn/Pose Rotation");
    logMax = new DoubleLogEntry(DataLogManager.getLog(), "Drivetrain:/GyroTurn/MaxSpeed");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Rotation2d rotation = new Rotation2d(Units.degreesToRadians(angle));
    if(DriverStation.getAlliance() == DriverStation.Alliance.Red)
    {
      rotation.times(-1);
    }
    Robot.drivetrain.putCommandString(this);
    controller.setSetpoint(rotation.getDegrees());
    logMax.append(maxSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turnValue = controller.calculate(Robot.drivetrain.getCurrentPose().getRotation().getDegrees());
    turnValue += Math.copySign(RobotMap.DrivetrainParameters.minTurnValue, turnValue);
    turnValue = Math.min(turnValue, maxSpeed);
    turnValue = Math.max(turnValue, -maxSpeed);
    Robot.drivetrain.arcadeDrive(0, turnValue);

    if(debug) {
      SmartDashboard.putNumber("Current Pose Degrees", Robot.drivetrain.getCurrentPose().getRotation().getDegrees());
      SmartDashboard.putNumber("Gyro Turn Setpoint", controller.getSetpoint());
      SmartDashboard.putNumber("Gyro Turn Output", turnValue);
    } else {
      logSetpoint.append(controller.getSetpoint());
      logCurrent.append(Robot.drivetrain.getCurrentPose().getRotation().getDegrees());
      logOutput.append(turnValue);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
