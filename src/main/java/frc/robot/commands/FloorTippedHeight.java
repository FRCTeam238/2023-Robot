// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.core238.MotionProfile;
import frc.core238.autonomous.AutonomousModeAnnotation;
import frc.robot.Robot;
import frc.robot.RobotMap;


@AutonomousModeAnnotation(parameterNames = {})
public class FloorTippedHeight extends ParallelCommandGroup {
  /** Creates a new MidHeight. */
  
  
  public FloorTippedHeight() { 
    Command ElevatorCube = new ElevatorProfile(new TrapezoidProfile.State(RobotMap.ElevatorParameters.cubeFloor, 0), "FloorCube");
    Command ElevatorCone = new ElevatorProfile(new TrapezoidProfile.State(RobotMap.ElevatorParameters.tippedConeFloor, 0), "FloorTipped");
    
    //AsProxy means that this command group doesn't inherit the requirements. This is important so that the subsystems
    //default commands will run after the setpoint is reached and hold the position while the other command completes
    //Conditional command means it will check the BooleanSupplier parameter at runtime and pick the right command
    addCommands(new ConditionalCommand(ElevatorCube.asProxy(), ElevatorCone.asProxy(), Robot::isCube));
    Command ArmCube = new ArmProfile(new MotionProfile.State(RobotMap.ArmParameters.cubeFloor), "FloorCube");
    Command ArmCone = new ArmProfile(new MotionProfile.State(RobotMap.ArmParameters.tippedConeFloor), "FloorTipped");
    addCommands(new ConditionalCommand(ArmCube.asProxy(), ArmCone.asProxy(), Robot::isCube));
  }
}
