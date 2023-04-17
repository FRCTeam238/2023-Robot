package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.core238.autonomous.AutonomousModeAnnotation;

@AutonomousModeAnnotation(parameterNames = {"time"})
public class DelayCommand extends CommandBase {
    double timeout;
    public DelayCommand(double timeout) {
        this.timeout = timeout;

    }


}