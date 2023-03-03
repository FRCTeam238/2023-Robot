// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.core238.autonomous.AutonomousModeAnnotation;
import frc.robot.Robot;

@AutonomousModeAnnotation(parameterNames = {"DistanceInches", "Speed"})
public class DriveStraightInches extends CommandBase implements IAutonomousCommand{

  private double startingVal;
  private double target;
  private double speed;

  /** Creates a new DriveStraightInches. */
  public DriveStraightInches() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.drivetrain);
  }

  public DriveStraightInches(double distanceInches, double speed) {
    target = distanceInches;
    this.speed = speed;
    addRequirements(Robot.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingVal = Units.metersToInches(Robot.drivetrain.stepsToMeters(Robot.drivetrain.getLeftEncoderTicks()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      Robot.drivetrain.tankDrive(speed, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("DONE DRIVING______________");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentDist = Units.metersToInches(Robot.drivetrain.stepsToMeters(Robot.drivetrain.getLeftEncoderTicks())) - startingVal;
    return Math.abs(currentDist) > target;
  }

  @Override
  public void setParameters(List<String> parameters) {
    // TODO Auto-generated method stub
    target = Double.parseDouble(parameters.get(0));
    speed = Double.parseDouble(parameters.get(1));
  }

  @Override
  public double getTimeout() {
    // TODO Auto-generated method stub
    return 0;
  }
}

