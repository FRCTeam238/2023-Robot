// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
 **/
public class LTVUnicycleCommand extends CommandBase {
    /**
     * Creates a new LTVUnicycleCommand.
     */

    private double startTime;
    private double currentTime;
    private final Supplier<Pose2d> m_pose;
    private final DifferentialDriveKinematics m_kinematics;
    private final BiConsumer<Double, Double> m_output;
    private PathPlannerTrajectory initialTrajectory;
    private PathPlannerTrajectory finalTrajectory;
    private boolean isFirstPath;
    LTVUnicycleController lu;
    private Field2d m_field;

    public LTVUnicycleCommand(PathPlannerTrajectory trajectory,
                              Supplier<Pose2d> pose,
                              DifferentialDriveKinematics kinematics,
                              BiConsumer<Double, Double> outputMetersPerSecond,
                              boolean isFirstPath,
                              SubsystemBase... requirements) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.initialTrajectory = trajectory;
        m_pose = pose;

        m_kinematics = kinematics;
        m_output = outputMetersPerSecond;
        this.isFirstPath = isFirstPath;

        m_field = new Field2d();
        
        // Shuffleboard.getTab("Logging").add("TrajectoryPose", m_field);
        addRequirements(requirements);
        lu = new LTVUnicycleController(0.02);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (isFirstPath) {
            PathPlannerState state = PathPlannerTrajectory.transformStateForAlliance(initialTrajectory.getInitialState(), DriverStation.getAlliance());
            Robot.drivetrain.resetOdometry(state.poseMeters);
            
        }
        startTime = Timer.getFPGATimestamp();
        
        finalTrajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(initialTrajectory, DriverStation.getAlliance());

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        currentTime = Timer.getFPGATimestamp() - startTime;
        SmartDashboard.putNumber("StateV", finalTrajectory.sample(currentTime).velocityMetersPerSecond);
        DifferentialDriveWheelSpeeds targetWheelSpeeds = m_kinematics.toWheelSpeeds(lu.calculate(m_pose.get(), finalTrajectory.sample(currentTime)));
        double leftOutput = targetWheelSpeeds.leftMetersPerSecond;
        double rightOutput = targetWheelSpeeds.rightMetersPerSecond;
        // should call the method given as the parameter for the BiConsumer
        m_output.accept(leftOutput, rightOutput);
        m_field.setRobotPose(finalTrajectory.sample(currentTime).poseMeters);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("END_______________________________________________");
        Robot.drivetrain.tankDrive(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        Pose2d diff = finalTrajectory.getEndState().poseMeters.relativeTo(m_pose.get());
        if (Math.abs(diff.getX()) < RobotMap.DrivetrainParameters.maxXTolerance) {
            if (Math.abs(diff.getY()) < RobotMap.DrivetrainParameters.maxYTolerance) {
                if (Math.abs(diff.getRotation().getDegrees()) < RobotMap.DrivetrainParameters.maxAngle) {
                    System.out.println("DONE___________________________________________");
                    return true;
                }
            }
        }
        return false;
    }
}
