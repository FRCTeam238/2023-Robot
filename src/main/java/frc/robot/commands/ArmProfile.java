// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmProfile extends CommandBase {
  /** Creates a new ArmProfile. */
  public ArmProfile(String name) {
    //TODO Should take a MotionProfile.State as a parameter and save it to a class variable, this is where the arm is trying to go
    //TODO require arm after it gets added to Robot
    //TODO create MotionConstraints object and add constants for it to RobotMap
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
