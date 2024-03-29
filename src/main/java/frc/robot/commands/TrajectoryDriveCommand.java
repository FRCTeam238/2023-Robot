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
import frc.core238.wrappers.ReverseChecker;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
@AutonomousModeAnnotation(parameterNames = {"TrajectoryName", "IsFirstPath", "MaxVelocity"})
public class TrajectoryDriveCommand extends SequentialCommandGroup implements IAutonomousCommand{
  DifferentialDriveKinematics kinematics;
  Drivetrain drivetrain = Robot.drivetrain;
  PathPlannerTrajectory trajectory;
  private boolean isFirstPath;
  /** Creates a new TrajectoryDriveCommand. */
  public TrajectoryDriveCommand() {
    kinematics = Drivetrain.kinematics;


    addRequirements(Robot.drivetrain);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
  }
  
  @Override
  public void setParameters(List<String> parameters) {
    // TODO Auto-generated method stub
    isFirstPath = Boolean.parseBoolean(parameters.get(1));
    boolean isReversed = ReverseChecker.checkReversed(parameters.get(0));
    double velocity = 0;
    try {
      velocity = Double.parseDouble(parameters.get(2));
    } catch (Exception e) {
      velocity = RobotMap.DrivetrainParameters.maxVelocity;
    }
    System.out.println("Velocity: " + velocity);
    trajectory = PathPlanner.loadPath(parameters.get(0), velocity, RobotMap.DrivetrainParameters.maxAccel, isReversed);
    TrajectoryControllerCommand ltv = new TrajectoryControllerCommand(trajectory, drivetrain::getCurrentPose, kinematics, isFirstPath, drivetrain);
    addCommands(ltv);
    
  }
  @Override
  public double getTimeout() {
    // TODO Auto-generated method stub
    return 0;
  }
}
