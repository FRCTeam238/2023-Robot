// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator;

public class ElevatorManualCommand extends CommandBase {
  Elevator elevator;

  private double position;
  /** Creates a new ElevatorManualCommand. */
  public ElevatorManualCommand() {
    elevator = Robot.elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.putCommandString(this);
    position = elevator.getEncoderPosition();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(RobotMap.ControlParameters.operatorController.getRightY()) > RobotMap.ControlParameters.elevatorThreshold) {
      double percentOutput = -1*RobotMap.ControlParameters.operatorController.getRightY() * RobotMap.ControlParameters.elevatorMultiplier;
        elevator.moveByPercentOutput(percentOutput);
        position = elevator.getEncoderPosition();
    } else if (elevator.getEncoderPosition() > RobotMap.ElevatorParameters.bottomThreshold) { // If we're at the bottom just let it go, otherwise hold

      elevator.holdPosition(position);
    } else {
      elevator.moveByPercentOutput(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
