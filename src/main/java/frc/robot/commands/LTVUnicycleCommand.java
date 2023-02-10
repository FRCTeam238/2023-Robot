// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
/**
 * a command that uses an {@link LTVUnicycleController} to follow
 * a {@link PathPlannerTrajectory}
 * 
 * <p>
 * while this could be modified to include PIDs, we did not need it,
 * so we didnt make it.
 *
 **/
public class LTVUnicycleCommand extends CommandBase {
  /** Creates a new LTVUnicycleCommand. */
  private String trajName;
  private double startTime;
  private double currentTime;
  private final Supplier<Pose2d> m_pose;
  private final DifferentialDriveKinematics m_kinematics;
  private final BiConsumer<Double, Double> m_output;
  PathPlannerTrajectory trajectory;
  
  public LTVUnicycleCommand(String trajectoryName,
			    Supplier<Pose2d> pose,
			    DifferentialDriveKinematics kinematics,
			    BiConsumer<Double, Double> outputMetersPerSecond,
			    SubsystemBase... requirements) {
    // Use addRequirements() here to declare subsystem dependencies.
    trajName = trajectoryName;
    m_pose = pose;
    
    m_kinematics = kinematics;
    m_output = outputMetersPerSecond;
    addRequirements(requirements);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    trajectory = PathPlanner.loadPath(trajName, RobotMap.DrivetrainParameters.maxVelocity, RobotMap.DrivetrainParameters.maxAccel);
  
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    	currentTime = Timer.getFPGATimestamp() - startTime;
    
    	LTVUnicycleController lu = new LTVUnicycleController(0.02);
	DifferentialDriveWheelSpeeds targetWheelSpeeds = m_kinematics.toWheelSpeeds(lu.calculate(m_pose.get(), trajectory.sample(currentTime)));
      	double leftOutput = targetWheelSpeeds.leftMetersPerSecond;
      	double rightOutput = targetWheelSpeeds.rightMetersPerSecond;
      	// should call the method given as the parameter for the BiConsumer
      	m_output.accept(leftOutput, rightOutput);

    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.drivetrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Pose2d diff = trajectory.getEndState().poseMeters.relativeTo(m_pose.get());
    if (Math.abs(diff.getX()) < RobotMap.DrivetrainParameters.maxXTolerance) {
      if (Math.abs(diff.getY()) < RobotMap.DrivetrainParameters.maxYTolerance) {
        if (Math.abs(diff.getRotation().getDegrees()) < RobotMap.DrivetrainParameters.maxAngle) {
          return true;
        }
      }
    }
    return false;
  }
}
