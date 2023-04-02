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
public class SingleSubHeight extends ParallelCommandGroup implements IAutonomousCommand {

    public SingleSubHeight() {
        Command ElevatorCone = new ElevatorTrapezoid(new TrapezoidProfile.State(RobotMap.ElevatorParameters.singleSubCone, 0), "SingleSubCone");
        Command ElevatorCube = new ElevatorTrapezoid(new TrapezoidProfile.State(RobotMap.ElevatorParameters.cubeFloor, 0), "SingleSubCube");
        addCommands(new ConditionalCommand(ElevatorCube.asProxy(), ElevatorCone.asProxy(), Robot::isCube));

        Command ArmCube = new ArmProfile(new MotionProfile.State(RobotMap.ArmParameters.cubeFloor, 0), "SingleSubCube");
        Command ArmCone = new ArmProfile(new MotionProfile.State(RobotMap.ArmParameters.singleSubCone, 0), "FloorStanding");
        addCommands(new ConditionalCommand(ArmCube.asProxy(), ArmCone.asProxy(), Robot::isCube));
    }

    @Override
    public void setParameters(List<String> parameters) {

    }

    @Override
    public double getTimeout() {
        return 0;
    }
}
