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
public class TrajectoryDriveCommand extends SequentialCommandGroup {
  DifferentialDriveKinematics kinematics;
  Drivetrain drivetrain = Robot.drivetrain;
  PathPlannerTrajectory trajectory;
  private boolean isFirstPath;


  /**
   * loads the path corresponding to the given <strong>trajectoryName</strong>
   * and builds a new {@link TrajectoryControllerCommand} with it
   *
   * @param trajectoryName : name of the trajectory to find and run
   * @param isFirstPath : if this path is being run first,
   * or if you need to replace the robot's state with the one from the start of the given path
   * @param maxVelocity the override for the maximum velocity of this path
   */
  public TrajectoryDriveCommand(String trajectoryName, boolean isFirstPath, double maxVelocity) {
    // TODO Auto-generated method stub
    addRequirements(Robot.drivetrain);
    kinematics = Drivetrain.kinematics;
    boolean isReversed = ReverseChecker.checkReversed(trajectoryName);
    trajectory = PathPlanner.loadPath(trajectoryName, maxVelocity, RobotMap.DrivetrainParameters.maxAccel, isReversed);
    TrajectoryControllerCommand ltv = new TrajectoryControllerCommand(trajectory, drivetrain::getCurrentPose, kinematics, isFirstPath, drivetrain);
    addCommands(ltv);

  }

  /**
   * backup constructor in case a maximum velocity is not given
   *
   * @param trajectoryName : name of the trajectory to find and run
   * @param isFirstPath : if this path is being run first,
   * or if you need to replace the robot's state with the one from the start of the given path
   */
  public TrajectoryDriveCommand(String trajectoryName, boolean isFirstPath) {
    // TODO Auto-generated method stub
    addRequirements(Robot.drivetrain);
    kinematics = Drivetrain.kinematics;
    boolean isReversed = ReverseChecker.checkReversed(trajectoryName);
    double velocity = RobotMap.DrivetrainParameters.maxVelocity;
    trajectory = PathPlanner.loadPath(trajectoryName, velocity, RobotMap.DrivetrainParameters.maxAccel, isReversed);
    TrajectoryControllerCommand ltv = new TrajectoryControllerCommand(trajectory, drivetrain::getCurrentPose, kinematics, isFirstPath, drivetrain);
    addCommands(ltv);

  }
}
