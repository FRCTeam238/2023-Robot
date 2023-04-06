// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.core238.autonomous.AutonomousModeAnnotation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
@AutonomousModeAnnotation(parameterNames = {})
public class GrabAuto extends SequentialCommandGroup implements IAutonomousCommand {
  /** Creates a new GrabAuto. */
  List<String> params = new ArrayList<String>();

  public GrabAuto() {
    addCommands(new FloorTippedHeight().asProxy());
    addCommands(new DriveStraightInches(18, .165).deadlineWith(new IntakeInOutCommand(IntakeInOutCommand.Direction.In).asProxy()));
  }
  
  @Override
  public void setParameters(List<String> parameters) {

  }

  @Override
  public double getTimeout() {
    return 0;
  }
}
