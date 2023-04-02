// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap.ControlParameters;

public class ArmManualCommand extends CommandBase {
  
  double position;

  /** Creates a new ArmManualCommand. */
  public ArmManualCommand() { 
  
   // Use addRequirements() here to declare subsystem dependencies.
   addRequirements(Robot.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    position = Robot.arm.getEncoderPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(ControlParameters.operatorController.getLeftY()) > ControlParameters.armThreshold) {
      Robot.arm.moveArmPercent(ControlParameters.operatorController.getLeftY() * ControlParameters.elevatorMultiplier);
      position = Robot.arm.getEncoderPosition();
    }
    else {
      Robot.arm.holdPosition(position);
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
