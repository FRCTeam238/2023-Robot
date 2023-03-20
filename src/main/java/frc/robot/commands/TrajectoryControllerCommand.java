// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * a command that uses an {@link LTVUnicycleController} to follow
 * a {@link PathPlannerTrajectory}
 *
 * <p>
 * while this could be modified to include PIDs, we did not need it,
 * so we didnt make it.
 **/
public class TrajectoryControllerCommand extends CommandBase {
    /**
     * Creates a new LTVUnicycleCommand.
     */

    private double startTime;
    private double currentTime;
    private final Supplier<Pose2d> m_pose;
    private final DifferentialDriveKinematics m_kinematics;
    private PathPlannerTrajectory initialTrajectory;
    private PathPlannerTrajectory finalTrajectory;
    private boolean isFirstPath;
    LTVUnicycleController lu;
    RamseteController rc;
    private Field2d m_field;
    private ControllerType type = ControllerType.LTV;

    public enum ControllerType{
        LTV,
        RAMSETE,
        NONE
    }

    public TrajectoryControllerCommand(PathPlannerTrajectory trajectory,
                              Supplier<Pose2d> pose,
                              DifferentialDriveKinematics kinematics,
                              boolean isFirstPath,
                              SubsystemBase... requirements) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.initialTrajectory = trajectory;
        m_pose = pose;

        m_kinematics = kinematics;
        this.isFirstPath = isFirstPath;

        m_field = new Field2d();
        
        addRequirements(requirements);
        lu = new LTVUnicycleController(0.02);
        rc = new RamseteController(3,.7);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        SmartDashboard.putData("TrajectoryPose", m_field);
        Robot.drivetrain.putCommandString(this);
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
        DifferentialDriveWheelSpeeds targetWheelSpeeds;
        DifferentialDriveWheelSpeeds nextWheelSpeeds;
        switch(type){
            case LTV:
                targetWheelSpeeds = m_kinematics.toWheelSpeeds(lu.calculate(m_pose.get(), finalTrajectory.sample(currentTime)));
                nextWheelSpeeds = m_kinematics.toWheelSpeeds(lu.calculate(m_pose.get(), finalTrajectory.sample(currentTime+.02)));
                break;
            case RAMSETE:
                targetWheelSpeeds = m_kinematics.toWheelSpeeds(rc.calculate(m_pose.get(), finalTrajectory.sample(currentTime)));
                nextWheelSpeeds = m_kinematics.toWheelSpeeds(rc.calculate(m_pose.get(), finalTrajectory.sample(currentTime+.02)));
                break;
            case NONE:
            default:
                State desiredState = finalTrajectory.sample(currentTime);
                targetWheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(desiredState.velocityMetersPerSecond, 0, desiredState.velocityMetersPerSecond * desiredState.curvatureRadPerMeter));
                desiredState = finalTrajectory.sample(currentTime+.02);
                nextWheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(desiredState.velocityMetersPerSecond, 0, desiredState.velocityMetersPerSecond * desiredState.curvatureRadPerMeter));
        }
         
        double leftOutput = targetWheelSpeeds.leftMetersPerSecond;
        double rightOutput = targetWheelSpeeds.rightMetersPerSecond;
        double leftAccel = (nextWheelSpeeds.leftMetersPerSecond - leftOutput)/.02;
        double rightAccel = (nextWheelSpeeds.rightMetersPerSecond - rightOutput)/.02;
        // should call the method given as the parameter for the BiConsumer
        Robot.drivetrain.driveByVelocityOutput(leftOutput, rightOutput, leftAccel, rightAccel);
        m_field.setRobotPose(finalTrajectory.sample(currentTime).poseMeters);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.drivetrain.tankDrive(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //If time is less than the trajectory time we're not done yet. This prevents false stopping if a path doubles back
        if(currentTime < finalTrajectory.getTotalTimeSeconds()){
            return false;
        }

        //Otherwise, check if our actual state is close enough to where we want to be.
        Pose2d diff = finalTrajectory.getEndState().poseMeters.relativeTo(m_pose.get());
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
