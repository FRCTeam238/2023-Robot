// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.core238.autonomous.AutonomousModeAnnotation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

@AutonomousModeAnnotation(parameterNames = {})
public class ReleaseGamepieceCommand extends SequentialCommandGroup {
  /** Creates a new ReleaseGamepieceCommand. */
  public ReleaseGamepieceCommand() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new IntakeInOutCommand(IntakeInOutCommand.Direction.OUT).withTimeout(1).asProxy());
  }
}
