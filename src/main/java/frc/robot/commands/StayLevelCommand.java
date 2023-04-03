// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.core238.autonomous.AutonomousModeAnnotation;
import frc.robot.Robot;

@AutonomousModeAnnotation(parameterNames = {})
public class StayLevelCommand extends CommandBase implements IAutonomousCommand {
  /** Creates a new StayLevelCommand. */

  final double triggerAngle = 12;
  final double driveSpeed = .09;
  final double finishAngle = 5;
  double lastPitchValue = 0;

  public StayLevelCommand() {
    addRequirements(Robot.drivetrain);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.drivetrain.putCommandString(this);
    lastPitchValue = Robot.drivetrain.getPitch();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(Robot.drivetrain.getPitch()) < triggerAngle) {
      Robot.drivetrain.arcadeDrive(0, 0);
      
    } else {
      
      Robot.drivetrain.arcadeDrive(Math.copySign(driveSpeed, Robot.drivetrain.getPitch()), 0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*if (Math.abs(Robot.drivetrain.getPitch())  < finishAngle && Math.abs(lastPitchValue - Robot.drivetrain.getPitch()) < 0.1) {
      return true;
    }



    lastPitchValue = Robot.drivetrain.getPitch();
    return false;*/
    return false;
  }

  @Override
  public void setParameters(List<String> parameters) {
    // TODO Auto-generated method stub
    
  }

  @Override
  public double getTimeout() {
    // TODO Auto-generated method stub
    return 0;
  }
}
