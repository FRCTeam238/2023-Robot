// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class KindaRunIntakeCommand extends CommandBase {
  RobotMap.IntakeParameters.Gamepiece gamepiece = Robot.gamepiece;

  /** Creates a new KindaRunIntakeCommand. */
  public KindaRunIntakeCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.intake.putCommandString(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gamepiece = Robot.gamepiece;

    Robot.intake.run(gamepiece == RobotMap.IntakeParameters.Gamepiece.CONE ? RobotMap.IntakeParameters.holdSpeedCone 
    : RobotMap.IntakeParameters.holdSpeedCube*-1);

  }

}
