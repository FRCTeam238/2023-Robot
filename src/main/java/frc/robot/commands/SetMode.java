// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.fasterxml.jackson.databind.deser.impl.CreatorCandidate.Param;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.core238.autonomous.AutonomousModeAnnotation;
import frc.robot.Robot;
import frc.robot.RobotMap.IntakeParameters.Gamepiece;

@AutonomousModeAnnotation(parameterNames = {"Gamepiece"})
public class SetMode extends InstantCommand implements IAutonomousCommand{
  Gamepiece gamepiece;

  /** Creates a new SetMode. */
  public SetMode(Gamepiece gamepiece) {
    this.gamepiece = gamepiece;
  }

  public SetMode() {}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.gamepiece = gamepiece;
  }

  @Override
  public void setParameters(List<String> parameters) {
    gamepiece = Gamepiece.valueOf(parameters.get(0).toUpperCase());
  }

  @Override
  public double getTimeout() {
    // TODO Auto-generated method stub
    return 0;
  }
}