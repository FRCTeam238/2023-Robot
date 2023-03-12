// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.core238.autonomous.AutonomousModeAnnotation;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

@AutonomousModeAnnotation(parameterNames = {})
public class StayLevelCommand extends CommandBase implements IAutonomousCommand {
  /** Creates a new StayLevelCommand. */

  double lastAngle = 0;
  public StayLevelCommand() {
    addRequirements(Robot.drivetrain);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.drivetrain.putCommandString(this);
    lastAngle = Robot.drivetrain.getPitch();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.drivetrain.tankDrive(0.12, 0.12);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(Robot.drivetrain.getRoll())  < 12) {
      return true;
    }
    lastAngle = Robot.drivetrain.getRoll();
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
