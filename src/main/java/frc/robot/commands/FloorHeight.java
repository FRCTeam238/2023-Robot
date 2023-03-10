// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.core238.autonomous.AutonomousModeAnnotation;
import frc.robot.Robot;
import frc.robot.RobotMap;

@AutonomousModeAnnotation(parameterNames = {})
public class FloorHeight extends ElevatorTrapezoid implements IAutonomousCommand{
  /** Creates a new MidHeight. */
  
  
  public FloorHeight() { 
    
    super(new TrapezoidProfile.State(RobotMap.ElevatorParameters.floorHeight, 0));
    // Use addRequirements() here to declare subsystem dependencies.
  }
  
  @Override
  public void initialize() {
    Robot.elevator.putCommandString(this);
    super.initialize();
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
