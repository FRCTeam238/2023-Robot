// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//Probably should have 4 different speeds in case they need to be different
//Intake speed and outtake speed for both types

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.core238.autonomous.AutonomousModeAnnotation;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotMap.IntakeParameters.Gamepiece;
import frc.robot.subsystems.Intake;

@AutonomousModeAnnotation(parameterNames = {})
public class IntakeInOutCommand extends CommandBase {
  
  Direction direction;
  Intake intake;
  
  
  public IntakeInOutCommand(Direction direction) {
    this.direction = direction;
    intake = Robot.intake;
    setName(getName() + ": " + direction.name());

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.putCommandString(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Robot.intake.isStalling() && direction == Direction.In)
    {
      RobotMap.ControlParameters.operatorController.setRumble(RumbleType.kBothRumble, 1);
      intake.run(Robot.gamepiece == Gamepiece.CONE ? 
      RobotMap.IntakeParameters.intakeSpeedCube: 
      RobotMap.IntakeParameters.holdSpeedCube * -1);
    } else {
      RobotMap.ControlParameters.operatorController.setRumble(RumbleType.kBothRumble, 0);
      if (direction == Direction.In) {
        intake.run(Robot.gamepiece == Gamepiece.CONE ? 
        RobotMap.IntakeParameters.intakeSpeedCone : 
        RobotMap.IntakeParameters.intakeSpeedCube * -1 );
      } else {
        intake.run(Robot.gamepiece == Gamepiece.CONE ? 
        RobotMap.IntakeParameters.outtakeSpeedCone * -1:
        RobotMap.IntakeParameters.outtakeSpeedCube);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    RobotMap.ControlParameters.operatorController.setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
//    if (DriverStation.isAutonomous() && direction == Direction.In) {
//     return intake.isStalling();
//    }

    return false;
  }

  public enum Direction {
    In,
    Out
  }

}
