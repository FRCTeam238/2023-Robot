// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class TopHeight extends DriveToHeightSimple {
  /** Creates a new MidHeight. */
  public TopHeight() { 
    
    super(new TrapezoidProfile.State(RobotMap.ElevatorParameters.midConeHeight, 0));
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!Robot.intake.isEitherExtended()) {
      Robot.intake.extendShort();
    }
  }
}
