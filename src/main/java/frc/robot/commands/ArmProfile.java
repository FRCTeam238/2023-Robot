// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.core238.MotionProfile;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class ArmProfile extends CommandBase {

  private MotionProfile profile;
  private MotionProfile.State goal;
  private MotionProfile.MotionConstraints constraints;

  /** Creates a new ArmProfile. */
  public ArmProfile(MotionProfile.State goal, String name) {

    this.goal = goal;
    addRequirements(Robot.arm);
    constraints = new MotionProfile.MotionConstraints(RobotMap.ArmParameters.maxJerk, RobotMap.ArmParameters.maxAccel, RobotMap.ArmParameters.maxVelocity, RobotMap.ArmParameters.velocityTolerance);
    this.setName(name);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    MotionProfile.State currentState = new MotionProfile.State(Robot.arm.getEncoderPosition(), Robot.arm.getVelocity());

    profile = new MotionProfile(goal, currentState, constraints, MotionProfile.ProfileType.AUTO);
    Robot.arm.putCommandString(this);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //check if going to cube 3
    if(goal.position < 40 && goal.position > 20)
    {
      //if so, no-op until the elevator gets high enough
      if(!Robot.elevator.clearCube3())
        return;
    }
    MotionProfile.State sampleState = profile.sample();
    Robot.arm.moveArmVelocity(sampleState);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!profile.isFinished())
    {
      return false;
    }
    return Math.abs(goal.position - Robot.arm.getEncoderPosition()) <= RobotMap.ArmParameters.tolerance
            && Math.abs(goal.velocity - Robot.arm.getVelocity()) <= RobotMap.ArmParameters.velocityTolerance;
  }
}
