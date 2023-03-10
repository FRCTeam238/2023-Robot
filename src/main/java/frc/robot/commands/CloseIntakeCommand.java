// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Intake;

public class CloseIntakeCommand extends InstantCommand {
  /** Creates a new CloseIntakeCommand. */
  public CloseIntakeCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Intake intake = Robot.intake;
    addRequirements(intake);
    intake.closeIntake();
    intake.putCommandString(this);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

}
