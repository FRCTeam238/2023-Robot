// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//TODO: Rework this for new intake. Should check gamepiece and determine setpoints
//Then can set arm and elevator setpoints in parallel. Other setpoint commands should have same structure

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.*;
import frc.core238.MotionProfile;
import frc.core238.autonomous.AutonomousModeAnnotation;
import frc.robot.Robot;
import frc.robot.RobotMap;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
@AutonomousModeAnnotation(parameterNames = {})
public class StowCommand extends ParallelCommandGroup {
  /** Creates a new StowCommand. */
  public StowCommand() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Command ElevatorCone = new ElevatorProfile(new TrapezoidProfile.State(RobotMap.ElevatorParameters.tippedConeFloor, 0), "StowCone");
    Command ElevatorCube = new ElevatorProfile(new TrapezoidProfile.State(RobotMap.ElevatorParameters.tippedConeFloor, 0), "StowCube");
    addCommands(new ConditionalCommand(ElevatorCube.asProxy(), ElevatorCone.asProxy(), Robot::isCube));

    Command ArmCube = new ArmProfile(new MotionProfile.State(RobotMap.ArmParameters.stow), "StowCube");
    Command ArmCone = new ArmProfile(new MotionProfile.State(RobotMap.ArmParameters.stow), "Lvl1Cone");
    addCommands(new ConditionalCommand(ArmCube.asProxy(), ArmCone.asProxy(), Robot::isCube));
  }
}
