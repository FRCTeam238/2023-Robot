// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.core238.autonomous.AutonomousModeAnnotation;
import frc.robot.RobotMap;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
@AutonomousModeAnnotation(parameterNames = {})
public class GrabAutoCone extends SequentialCommandGroup implements IAutonomousCommand {
  /** Creates a new GrabAuto. */
  List<String> params = new ArrayList<String>();

  public GrabAutoCone() {
    // Add your commands in the addCommands() call, e.g.
    addCommands(new Armdown());
    addCommands(new WaitCommand(RobotMap.IntakeParameters.armDelay));
    addCommands(new OpenIntakeCommand());
    addCommands(new DriveStraightInches(6, .2));
    addCommands(new CloseIntakeCommand());
    // addCommands(new FooCommand(), new BarCommand());
    
  }
  
  @Override
  public void setParameters(List<String> parameters) {
    // TODO Auto-generated method stub
    
  }

  @Override
  public double getTimeout() {
    // TODO Auto-generated method stub
    return 0;
  }
}
