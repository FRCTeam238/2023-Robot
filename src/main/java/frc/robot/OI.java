package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.core238.DriverControls;
import frc.core238.DriverControls.driveType;
import frc.robot.RobotMap.IntakeParameters.Gamepiece;
import frc.robot.commands.*;
import frc.robot.commands.IntakeInOutCommand.Direction;

import javax.print.DocFlavor;

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
		
		//Right Bumper
		JoystickButton setCube = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
		setCube.onTrue(new SetMode(Gamepiece.CUBE));
		
		//Left Bumper
		JoystickButton setCone = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
		setCone.onTrue(new SetMode(Gamepiece.CONE));
		
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


		JoystickButton charge = new JoystickButton(leftJoystick, 5);
		charge.whileTrue(new StayLevelCommand());
	}
}
