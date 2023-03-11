package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;

import java.util.function.BiConsumer;

public class GeneratePathLive extends SequentialCommandGroup {
    DifferentialDriveKinematics kinematics;
    Drivetrain drivetrain = Robot.drivetrain;
    Translation2d endTranslation;
    Rotation2d endRotation;

    public GeneratePathLive(Translation2d endTranslation, Rotation2d endRotation) {
        this.endRotation = endRotation;
        this.endTranslation = endTranslation;
        kinematics = Drivetrain.kinematics;

        double posX = drivetrain.getCurrentPose().getX();
        double posY = drivetrain.getCurrentPose().getY();
        Rotation2d rot = drivetrain.getCurrentPose().getRotation();
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(new PathConstraints(RobotMap.DrivetrainParameters.maxVelocity,
                                                                                        RobotMap.DrivetrainParameters.maxAccel),
                                                                    new PathPoint(new Translation2d(posX, posY), rot),
                                                                    new PathPoint(endTranslation, endRotation));
        TrajectoryControllerCommand ltv = new TrajectoryControllerCommand(trajectory, drivetrain::getCurrentPose, kinematics, false, drivetrain);
    }
}