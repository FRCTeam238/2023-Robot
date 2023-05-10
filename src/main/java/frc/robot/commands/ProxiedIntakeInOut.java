package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.core238.autonomous.AutonomousModeAnnotation;

import java.util.List;

//TODO: this is awful, refactor autos in summer

@AutonomousModeAnnotation(parameterNames = {})
public class ProxiedIntakeInOut extends SequentialCommandGroup implements IAutonomousCommand {
    public ProxiedIntakeInOut() {
        addCommands(new IntakeInOutCommand(IntakeInOutCommand.Direction.In).asProxy());

    }

    @Override
    public void setParameters(List<String> parameters) {

    }

    @Override
    public double getTimeout() {
        return 0;
    }

}