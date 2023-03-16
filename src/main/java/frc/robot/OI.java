package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.core238.DriverControls;
import frc.core238.DriverControls.driveType;
import frc.robot.commands.Armdown;
import frc.robot.commands.CloseIntakeCommand;
import frc.robot.commands.Drive;
import frc.robot.commands.ElevatorManualCommand;
import frc.robot.commands.FlickCone3Command;
import frc.robot.commands.FloorHeight;
import frc.robot.commands.IntakeInOutCommand;
import frc.robot.commands.KindaRunIntakeCommand;
import frc.robot.commands.MidConeHeight;
import frc.robot.commands.MidCubeHeight;
import frc.robot.commands.OpenIntakeCommand;
import frc.robot.commands.Scoringposition;
import frc.robot.commands.SetBrakeCommand;
import frc.robot.commands.StowCommand;
import frc.robot.commands.TopHeight;
import frc.robot.commands.Travelposition;
import frc.robot.commands.IntakeInOutCommand.Direction;

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

		Robot.intake.setDefaultCommand(new KindaRunIntakeCommand());

		
		driveCommand = new Drive();
		elevatorManualCommand = new ElevatorManualCommand();


		Robot.drivetrain.setDefaultCommand(driveCommand);	
		Robot.elevator.setDefaultCommand(elevatorManualCommand);

		//Left Bumper
		JoystickButton closeIntake = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
		closeIntake.onTrue(new CloseIntakeCommand());

		//B Button
		JoystickButton midCone = new JoystickButton(operatorController, XboxController.Button.kB.value);
		midCone.onTrue(new MidConeHeight());
		
		//X Button
		JoystickButton midCube = new JoystickButton(operatorController, XboxController.Button.kX.value);
		midCube.onTrue(new MidCubeHeight());
		
		//A Button
		JoystickButton floorHeight = new JoystickButton(operatorController, XboxController.Button.kA.value);
		floorHeight.onTrue(new FloorHeight());
		
		//Y Button
		JoystickButton topHeight = new JoystickButton(operatorController, XboxController.Button.kY.value);
		topHeight.onTrue(new TopHeight());

		commandController.leftTrigger(.1).whileTrue(new IntakeInOutCommand(Direction.In));
		commandController.rightTrigger(.1).whileTrue(new IntakeInOutCommand(Direction.Out));

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
		stowPos.onTrue(new StowCommand().withTimeout(5));

		JoystickButton brakeModeButton = new JoystickButton(rightJoystick, 3);
		brakeModeButton.onTrue(new SetBrakeCommand());
		
		//Start button
		JoystickButton flickCube = new JoystickButton(operatorController, XboxController.Button.kStart.value);
		flickCube.onTrue(new FlickCone3Command());
	}
	
		
}
