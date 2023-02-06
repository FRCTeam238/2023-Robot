// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class DriveToHeight extends TrapezoidProfileCommand {
  /** Creates a new DriveToHeight. */
  public DriveToHeight(State endState) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(new TrapezoidProfile(new TrapezoidProfile.Constraints(RobotMap.ElevatorParameters.MaxVel, RobotMap.ElevatorParameters.MaxAccel), endState, 
    new TrapezoidProfile.State(Robot.elevator.getEncoderPosition(),Robot.elevator.getVelocity())), 
     setpointState -> Robot.elevator.PIDdrive(setpointState), Robot.elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
