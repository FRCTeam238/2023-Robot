package frc.robot.commands;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;

/**
 * ArcadeDrive
 */
public class ArcadeDrive {
	DifferentialDrive diff = new DifferentialDrive(null, null);
	diff.arcadeDrive();
	
}
