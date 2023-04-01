// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.core238.MotionProfile;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class ArmProfile extends CommandBase {

  private MotionProfile.State state;
  private MotionProfile.MotionConstraints constraints;

  /** Creates a new ArmProfile. */
  public ArmProfile(MotionProfile.State state, String name) {

    this.state = state;
    addRequirements(Robot.arm);
    constraints = new MotionProfile.MotionConstraints(RobotMap.ArmParameters.maxJerk, RobotMap.ArmParameters.maxAccel, RobotMap.ArmParameters.maxVelocity, RobotMap.ArmParameters.velocityTolerance);
    this.setName(name);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //TODO should construct it's own MotionProfile.State describing the current state
    //TODO Then create a MotionProfile object with those states and the MotionConstraints object
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO Should call sample() on MotionProfile and pass the state recieved to arm velocity drive
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //TODO Should check if we are at the goal, should be pretty similar to elevator
    return false;
  }
}
