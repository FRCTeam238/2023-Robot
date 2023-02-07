// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class MidHeight extends DriveToHeight {
  /** Creates a new MidHeight. */
  public MidHeight() {
    super(new TrapezoidProfile.State(Robot.intake.hasCube() ? RobotMap.ElevatorParameters.midCubeHeight : RobotMap.ElevatorParameters.midConeHeight, 0));
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
