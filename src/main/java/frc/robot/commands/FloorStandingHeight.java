package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.core238.MotionProfile;
import frc.core238.autonomous.AutonomousModeAnnotation;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Arm;

import java.util.List;

@AutonomousModeAnnotation(parameterNames = {})
public class FloorStandingHeight extends ParallelCommandGroup implements IAutonomousCommand {

    public FloorStandingHeight() {
        Command ElevatorCone = new ElevatorTrapezoid(new TrapezoidProfile.State(RobotMap.ElevatorParameters.standingCone, 0), "FloorStanding");
        Command ElevatorCube = new ElevatorTrapezoid(new TrapezoidProfile.State(RobotMap.ElevatorParameters.cubeFloor, 0), "FloorCube");
        addCommands(new ConditionalCommand(ElevatorCube.asProxy(), ElevatorCone.asProxy(), Robot::isCube));

        Command ArmCube = new ArmProfile(new MotionProfile.State(RobotMap.ArmParameters.cubeFloor, 0), "FloorCube");
        Command ArmCone = new ArmProfile(new MotionProfile.State(RobotMap.ArmParameters.standingCone, 0), "FloorStanding");
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
