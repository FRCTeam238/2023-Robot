// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.core238.MotionProfile;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator;

public class ElevatorProfile extends CommandBase {
  MotionProfile.State goal;
  MotionProfile.MotionConstraints constraints;
  Timer timer;
  MotionProfile profile;
  Elevator elevator = Robot.elevator;
 MotionProfile.State currentState;

  /** Creates a new Trapezoid238. */
  public ElevatorProfile(MotionProfile.State goal, String name) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.goal = goal;
    constraints = new MotionProfile.MotionConstraints(RobotMap.ElevatorParameters.MaxElevatorJerk, RobotMap.ElevatorParameters.MaxAccel, RobotMap.ElevatorParameters.MaxVel, RobotMap.ElevatorParameters.elevatorVelocityTolerance);
    elevator.putCommandString(this);
    addRequirements(Robot.elevator);
    this.setName(name);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentState = new MotionProfile.State(Robot.elevator.getEncoderPosition(), Robot.elevator.getVelocity());
    profile = new MotionProfile(goal, currentState, constraints, MotionProfile.ProfileType.AUTO);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Robot.arm.hitsBumper()) {
      timer.restart();
    } else {
      currentState = profile.sample();
      nextState = profile.sample();
      elevator.PIDdrive(currentState, nextState);

    }
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
