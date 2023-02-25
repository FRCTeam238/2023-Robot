// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.core238.autonomous.AutonomousModeAnnotation;
import frc.robot.Robot;
/**
 * {@summary moves to scoring position, waits 3 seconds, and opens the intake}
 */
@AutonomousModeAnnotation(parameterNames = {})
public class ScoreAuto extends SequentialCommandGroup implements IAutonomousCommand {
  /** Creates a new Wait238. */
  
  public ScoreAuto() {
    addRequirements(Robot.intake);
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(new Scoringposition());
    addCommands(new WaitCommand(2));
    addCommands(new OpenIntakeCommand());  
  }
  
  @Override
  public double getTimeout() {
      // TODO Auto-generated method stub
      return 0;
  }

  @Override
  public void setParameters(List<String> parameters) {
      // TODO Auto-generated method stub
      
  }

  // Called when the command is initially scheduled.
}
