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
public class DoubleSubHeight extends ParallelCommandGroup  {

    public DoubleSubHeight() {
        Command ElevatorCone = new ElevatorTrapezoid(new TrapezoidProfile.State(RobotMap.ElevatorParameters.doubleSubCone, 0), "DoubleSubCone");
        Command ElevatorCube = new ElevatorTrapezoid(new TrapezoidProfile.State(RobotMap.ElevatorParameters.doubleSubCube, 0), "DoubleSubCube");
        addCommands(new ConditionalCommand(ElevatorCube.asProxy(), ElevatorCone.asProxy(), Robot::isCube));

        Command ArmCube = new ArmProfile(new MotionProfile.State(RobotMap.ArmParameters.doubleSubCube), "DoubleSubCube");
        Command ArmCone = new ArmProfile(new MotionProfile.State(RobotMap.ArmParameters.doubleSubCone), "DoubleSubCone");
        addCommands(new ConditionalCommand(ArmCube.asProxy(), ArmCone.asProxy(), Robot::isCube));
    }

}
