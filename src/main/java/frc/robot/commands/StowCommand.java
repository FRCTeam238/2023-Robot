// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//TODO: Rework this for new intake. Should check gamepiece and determine setpoints
//Then can set arm and elevator setpoints in parallel. Other setpoint commands should have same structure

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.core238.autonomous.AutonomousModeAnnotation;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
@AutonomousModeAnnotation(parameterNames = {})
public class StowCommand extends SequentialCommandGroup implements IAutonomousCommand {
  /** Creates a new StowCommand. */
  public StowCommand() {
    Intake intake = Robot.intake;
    Elevator elevator = Robot.elevator;
    addRequirements(intake, elevator);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new Travelposition().unless(() -> elevator.getEncoderPosition() < 2));
    addCommands(new WaitCommand(.4).unless(() -> elevator.getEncoderPosition() < 2));
    addCommands(new FloorHeight());
    addCommands(new RetractAll());
  }
  @Override
  public double getTimeout() {
      // TODO Auto-generated method stub
      return 0;
  }
  @Override
  public void setParameters(List<String> parameters) {
      // TODO Auto-generated method stub
      
  }
}
