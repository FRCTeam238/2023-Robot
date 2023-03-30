// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator;

public class ElevatorTrapezoid extends CommandBase {
  State goal;
  Constraints constraints;
  Timer timer;
  TrapezoidProfile profile;
  Elevator elevator = Robot.elevator;
  State currentState, nextState;
  /** Creates a new Trapezoid238. */
  public ElevatorTrapezoid(State goal) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.goal = goal;
    constraints = new Constraints(RobotMap.ElevatorParameters.MaxVel, RobotMap.ElevatorParameters.MaxAccel);
    timer = new Timer();
    addRequirements(Robot.elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double posMeters = elevator.getEncoderPosition();
    double velocity = elevator.getVelocity();
    State startPos = new State(posMeters, velocity);
    profile = new TrapezoidProfile(constraints, goal, startPos);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentState = profile.calculate(timer.get());
    nextState = profile.calculate(timer.get() +.02);
    elevator.PIDdrive(currentState, nextState);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double tolerance = RobotMap.ElevatorParameters.toleranceRotations;
    double velocityTolerance = RobotMap.ElevatorParameters.toleranceVelocity;
    if (Math.abs(goal.position - elevator.getEncoderPosition()) <= tolerance 
    && Math.abs(goal.velocity - elevator.getVelocity()) <= velocityTolerance) {
      return true;
    }

    return false;

  }
}
