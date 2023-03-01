package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.core238.autonomous.AutonomousModeAnnotation;

import java.util.List;

@AutonomousModeAnnotation(parameterNames = {"time"})
public class DelayCommand extends CommandBase implements IAutonomousCommand {
    double timeout;
    public DelayCommand() {}

    @Override
    public void setParameters(List<String> parameters) {
        timeout = Double.parseDouble(parameters.get(0));
    }

    @Override
    public double getTimeout() {
        return timeout;
    }
}