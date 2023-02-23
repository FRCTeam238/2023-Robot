// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.core238.autonomous.AutonomousModeAnnotation;
import frc.robot.Robot;
import frc.robot.RobotMap;

@AutonomousModeAnnotation(parameterNames = { "angle", "maxSpeed" })
public class GyroTurnCommand extends CommandBase implements IAutonomousCommand {
  /** Creates a new GyroTurnCommand. */
  double angle;
  double maxSpeed;
  PIDController controller;

  public GyroTurnCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.drivetrain);
    controller = new PIDController(RobotMap.DrivetrainParameters.kPSpin, RobotMap.DrivetrainParameters.kISpin,
        RobotMap.DrivetrainParameters.kDSpin);
    controller.enableContinuousInput(0, 360);
    controller.setTolerance(RobotMap.DrivetrainParameters.angleTolerance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setSetpoint(angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turnValue = controller.calculate(Robot.drivetrain.getCurrentPose().getRotation().getDegrees());
    turnValue = Math.min(turnValue, maxSpeed);
    turnValue = Math.max(turnValue, -maxSpeed);
    Robot.drivetrain.arcadeDrive(0, turnValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }

  @Override
  public void setParameters(List<String> parameters) {
    // TODO Auto-generated method stub
    angle = Double.parseDouble(parameters.get(0));
    maxSpeed = Double.parseDouble(parameters.get(1));
  }

  @Override
  public double getTimeout() {
    // TODO Auto-generated method stub
    return 0;
  }

}