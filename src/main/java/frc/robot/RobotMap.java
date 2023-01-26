package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

/**
 * RobotMap
 */
public final class RobotMap {


	public static class Drivetrain {
        int rightDTLeaderID;
        int leftDTLeaderID;
        int rightDTFollowerID;
        int leftDTFollowerID;
	    TalonFX rightDrivetrainLeader = new TalonFX(rightDTLeaderID);
        TalonFX leftDrivetrainLeader = new TalonFX(leftDTLeaderID);
        TalonFX leftDrivetrainFollower = new TalonFX(leftDTFollowerID);
        TalonFX rightDrivetrainFollower = new TalonFX(rightDTFollowerID);
        
	}
}
