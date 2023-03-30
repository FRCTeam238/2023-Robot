// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//TODO: Reconnect to Intake. Needs to run intake in opposite direction depending on gamepiece type.
//Probably should have 4 different speeds in case they need to be different
//Intake speed and outtake speed for both types

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.core238.autonomous.AutonomousModeAnnotation;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.OldIntake;

@AutonomousModeAnnotation(parameterNames = {})
public class IntakeInOutCommand extends CommandBase {
  
  Direction direction;
  OldIntake intake;
  
  
  public IntakeInOutCommand(Direction direction) {
    this.direction = direction;
    intake = Robot.intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.putCommandString(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (direction == Direction.In) {
      intake.runIntake(-1*RobotMap.IntakeParameters.intakeSpeed);
    } else {
      intake.runIntake(RobotMap.IntakeParameters.intakeEjectSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.runIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public enum Direction {
    In,
    Out
  }

}
