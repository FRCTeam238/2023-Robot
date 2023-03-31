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


//TODO make all the other height commands based off how this one works
@AutonomousModeAnnotation(parameterNames = {})
public class FloorTippedHeight extends ParallelCommandGroup implements IAutonomousCommand{
  /** Creates a new MidHeight. */
  
  
  public FloorTippedHeight() { 
    Command ElevatorCube = new ElevatorTrapezoid(new TrapezoidProfile.State(RobotMap.ElevatorParameters.cubeFloor, 0), "FloorCube");
    Command ElevatorCone = new ElevatorTrapezoid(new TrapezoidProfile.State(RobotMap.ElevatorParameters.tippedConeFloor, 0), "FloorTipped");
    addCommands(new ConditionalCommand(ElevatorCube, ElevatorCone, Robot::isCube));
    Command ArmCube = new ArmProfile(new MotionProfile.State(RobotMap.ArmParameters.cubeFloor, 0), "FloorCube");
    Command ArmCone = new ArmProfile(new MotionProfile.State(RobotMap.ArmParameters.tippedConeFloor, 0), "FloorTipped");
    addCommands(new ConditionalCommand(ArmCube, ArmCone, Robot::isCube));
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
