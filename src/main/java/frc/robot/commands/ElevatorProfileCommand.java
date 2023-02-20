// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class ElevatorProfileCommand extends CommandBase {

  TrapezoidProfile profile;
  TrapezoidProfile.State goal;
  TrapezoidProfile.Constraints constraints;
  Timer timer;

  /** Creates a new ElevatorProfileCommand. */
  public ElevatorProfileCommand(State endState) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.elevator);
    goal = endState;
    constraints = new TrapezoidProfile.Constraints(RobotMap.ElevatorParameters.MaxVel, RobotMap.ElevatorParameters.MaxAccel);
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    TrapezoidProfile.State initial = new TrapezoidProfile.State(Robot.elevator.getEncoderPosition(), Robot.elevator.getVelocity());
    profile = new TrapezoidProfile(constraints, goal, initial);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.elevator.PIDdrive(profile.calculate(timer.get()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.elevator.moveByPercentOutput(RobotMap.ElevatorParameters.holdPercent);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(Robot.elevator.getEncoderPosition() - goal.position) < RobotMap.ElevatorParameters.maxPositionError && Math.abs(Robot.elevator.getVelocity()) < RobotMap.ElevatorParameters.maxVelocityError;
  }
}
