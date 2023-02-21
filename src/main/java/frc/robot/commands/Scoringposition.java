// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.core238.autonomous.AutonomousModeAnnotation;
import frc.robot.Robot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
@AutonomousModeAnnotation(parameterNames = {})
public class Scoringposition extends InstantCommand implements IAutonomousCommand{
  public Scoringposition() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.intake.extendLong();
    Robot.intake.retractShort();
  }
  @Override
  public boolean getIsAutonomousMode() {
      // TODO Auto-generated method stub
      return false;
  }
  @Override
  public double getTimeout() {
      // TODO Auto-generated method stub
      return 0;
  }
  @Override
  public void setIsAutonomousMode(boolean isAutonomousMode) {
      // TODO Auto-generated method stub
      
  }
  @Override
  public void setParameters(List<String> parameters) {
      // TODO Auto-generated method stub
      
  }
}
