package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.core238.MotionProfile;
import frc.core238.autonomous.AutonomousModeAnnotation;
import frc.robot.Robot;
import frc.robot.RobotMap;

import java.util.List;

@AutonomousModeAnnotation(parameterNames = {})
public class LevelThreeHeight extends ParallelCommandGroup {
    double timeout;

    public LevelThreeHeight() {
        Command ElevatorCone = new ElevatorProfile(new MotionProfile.State(RobotMap.ElevatorParameters.coneLevel3, 0), "Lvl3Cone");
        Command ElevatorCube = new ElevatorProfile(new MotionProfile.State(RobotMap.ElevatorParameters.cubeLevel3, 0), "Lvl3Cube");
        addCommands(new ConditionalCommand(ElevatorCube.asProxy(), ElevatorCone.asProxy(), Robot::isCube));

        Command ArmCube = new ArmProfile(new MotionProfile.State(RobotMap.ArmParameters.cubeLevel3), "Lvl3Cube");
        Command ArmCone = new ArmProfile(new MotionProfile.State(RobotMap.ArmParameters.coneLevel3), "Lvl3Cone");
        addCommands(new ConditionalCommand(ArmCube.asProxy(), ArmCone.asProxy(), Robot::isCube));
    }
}
