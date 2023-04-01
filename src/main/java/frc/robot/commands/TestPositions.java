package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.core238.MotionProfile;

import java.util.function.Supplier;

public class TestPositions extends CommandBase {
    Supplier<Double> position;
    public TestPositions(Supplier<Double> position) {
        this.position = position;
    }

    @Override
    public void initialize() {
        ArmProfile profile = new ArmProfile(new MotionProfile.State(position.get()), "asdsadasd");
        profile.schedule();

    }
}
