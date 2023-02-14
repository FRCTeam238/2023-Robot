// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.function.BiConsumer;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.core238.autonomous.AutonomousModeAnnotation;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
@AutonomousModeAnnotation(parameterNames = {"TrajectoryName"})
public class TrajectoryDriveCommand extends SequentialCommandGroup implements IAutonomousCommand{
  DifferentialDriveKinematics kinematics;
  Drivetrain drivetrain = Robot.drivetrain;
  PathPlannerTrajectory trajectory;
  BiConsumer<Double, Double> output;
  /** Creates a new TrajectoryDriveCommand. */
  public TrajectoryDriveCommand() {
    kinematics = Drivetrain.kinematics;
    output = drivetrain::driveByVelocityOutput;


    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
  }
  
  @Override
  public boolean getIsAutonomousMode() {
    // TODO Auto-generated method stub
    return false;
  }
  
  @Override
  public void setIsAutonomousMode(boolean isAutonomousMode) {
    // TODO Auto-generated method stub
    
  }
  @Override
  public void setParameters(List<String> parameters) {
    // TODO Auto-generated method stub
    trajectory = PathPlanner.loadPath(parameters.get(0), RobotMap.DrivetrainParameters.maxVelocity, RobotMap.DrivetrainParameters.maxAccel);
    LTVUnicycleCommand ltv = new LTVUnicycleCommand(trajectory, drivetrain::getCurrentPose, kinematics, output, drivetrain);
    addCommands(ltv);
    
  }
  @Override
  public double getTimeout() {
    // TODO Auto-generated method stub
    return 0;
  }
}
