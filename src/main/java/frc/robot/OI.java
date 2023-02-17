package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.core238.DriverControls;
import frc.core238.DriverControls.driveType;
import frc.core238.wrappers.TriggerButton;
import frc.robot.commands.Armdown;
import frc.robot.commands.CloseIntakeCommand;
import frc.robot.commands.Drive;
import frc.robot.commands.ElevatorManualCommand;
import frc.robot.commands.FloorHeight;
import frc.robot.commands.IntakeInOutCommand;
import frc.robot.commands.MidConeHeight;
import frc.robot.commands.MidCubeHeight;
import frc.robot.commands.OpenIntakeCommand;
import frc.robot.commands.ReleaseGamepieceCommand;
import frc.robot.commands.Scoringposition;
import frc.robot.commands.StowCommand;
import frc.robot.commands.TopHeight;
import frc.robot.commands.Travelposition;

/**
 * OI
 * stands for operatorInterface
 */
public class OI {


	Joystick leftJoystick = RobotMap.ControlParameters.left;
	Joystick rightJoystick = RobotMap.ControlParameters.right;
	XboxController operatorController = RobotMap.ControlParameters.operatorController;
	CommandXboxController commandController = new CommandXboxController(operatorController.getPort());
	DriverControls control = new DriverControls(leftJoystick, rightJoystick);

	ElevatorManualCommand elevatorManualCommand;
	Drive driveCommand;

	public OI(driveType controlType) {
		
		driveCommand = new Drive();
		elevatorManualCommand = new ElevatorManualCommand();


		Robot.drivetrain.setDefaultCommand(driveCommand);	
		Robot.elevator.setDefaultCommand(elevatorManualCommand);

		JoystickButton closeIntake = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
		closeIntake.onTrue(new CloseIntakeCommand());

		JoystickButton midCone = new JoystickButton(operatorController, XboxController.Button.kB.value);
		midCone.onTrue(new MidConeHeight());

		JoystickButton midCube = new JoystickButton(operatorController, XboxController.Button.kX.value);
		midCube.onTrue(new MidCubeHeight());

		JoystickButton floorHeight = new JoystickButton(operatorController, XboxController.Button.kA.value);
		floorHeight.onTrue(new FloorHeight());
		
		JoystickButton topHeight = new JoystickButton(operatorController, XboxController.Button.kY.value);
		topHeight.onTrue(new TopHeight());

		commandController.leftTrigger(.1).whileTrue(new IntakeInOutCommand(true));
		commandController.rightTrigger(.1).whileTrue(new IntakeInOutCommand(false));
		
		JoystickButton dropGamepiece = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
		dropGamepiece.whileTrue(new OpenIntakeCommand());
		
		//Dpad up
		POVButton Travelposition = new POVButton(operatorController, 0);
		Travelposition.onTrue(new Travelposition());

		//Dpad left & right
		POVButton Scoringposition = new POVButton(operatorController, 90);
		Scoringposition.onTrue(new Scoringposition());

		POVButton Scoringposition2 = new POVButton(operatorController, 270);
		Scoringposition2.onTrue(new Scoringposition());

		//Dpad down
		POVButton Armdown = new POVButton(operatorController, 180);
		Armdown.onTrue(new Armdown());

		//Back button
		JoystickButton stowPos = new JoystickButton(operatorController, XboxController.Button.kBack.value);
		stowPos.onTrue(new StowCommand());
	}
	
		
}
