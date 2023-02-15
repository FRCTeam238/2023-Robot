// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class DriveToHeightSimple extends CommandBase {
  PIDController controller;

  /** Creates a new DriveToHeight. */
  public DriveToHeightSimple(State endState) {
    addRequirements(Robot.elevator);
    controller = new PIDController(0.001, 0, 0); // .005 = Approximately 4V for 20in error
    controller.setSetpoint(endState.position);
    controller.setTolerance(18); // 1/2"
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.elevator.moveByPercentOutput(controller.calculate(Robot.elevator.getEncoderPosition()) + RobotMap.ElevatorParameters.holdPercent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}