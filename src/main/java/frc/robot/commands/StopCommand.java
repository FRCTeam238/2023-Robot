package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 * StopCommand uses PIDs to keep us in place no matter the angle 
 */
public class StopCommand extends CommandBase {

  Drivetrain drivetrain;
  public StopCommand(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }
  @Override
  public void initialize() {
    addRequirements(drivetrain);
  }
 
  @Override
  public void execute() {
    drivetrain.driveByVelocityOutput(0, 0); 
  }
  
}
