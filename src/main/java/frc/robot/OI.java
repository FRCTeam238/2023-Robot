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

/**
 * OI
 * stands for operatorInterface
 */
public class OI {


	Joystick leftJoystick = RobotMap.ControlParameters.left;
//	SendableChooser<Double> testArmChooser;
	Joystick rightJoystick = RobotMap.ControlParameters.right;
	XboxController operatorController = RobotMap.ControlParameters.operatorController;
	CommandXboxController commandController = new CommandXboxController(operatorController.getPort());
	DriverControls control = new DriverControls(leftJoystick, rightJoystick);


	ElevatorManualCommand elevatorManualCommand;
	Drive driveCommand;

	public OI(driveType controlType) {

		driveCommand = new Drive();
//		testArmChooser = new SendableChooser<>();
//		testArmChooser.addOption("stow", RobotMap.ArmParameters.stow);
//		testArmChooser.setDefaultOption("doubleSub", RobotMap.ArmParameters.doubleSubCube);
//		testArmChooser.addOption("floorCube", RobotMap.ArmParameters.cubeFloor);
//		SmartDashboard.putData("ArmPositions", testArmChooser);
		elevatorManualCommand = new ElevatorManualCommand();

		Robot.drivetrain.setDefaultCommand(driveCommand);
		Robot.elevator.setDefaultCommand(elevatorManualCommand);
		Robot.intake.setDefaultCommand(new KindaRunIntakeCommand());
		Robot.arm.setDefaultCommand(new ArmManualCommand());

		//Start button
		JoystickButton disablePID = new JoystickButton(operatorController, XboxController.Button.kStart.value);
		disablePID.onTrue(new DisablePID());

		//B Button
		JoystickButton levelTwoHeightB = new JoystickButton(operatorController, XboxController.Button.kB.value);
		levelTwoHeightB.onTrue(new LevelTwoHeight().withTimeout(3));

		//X Button
		JoystickButton levelTwoHeightX = new JoystickButton(operatorController, XboxController.Button.kX.value);
		levelTwoHeightX.onTrue(new LevelTwoHeight().withTimeout(3));

		//A Button
		JoystickButton levelOneHeight = new JoystickButton(operatorController, XboxController.Button.kA.value);
		levelOneHeight.onTrue(new LevelOneHeight().withTimeout(3));

		//A Button test
//		JoystickButton testHeight = new JoystickButton(operatorController, XboxController.Button.kA.value);
//		testHeight.onTrue(new TestPositions(testArmChooser::getSelected));

		//Y Button
		JoystickButton levelThreeHeight = new JoystickButton(operatorController, XboxController.Button.kY.value);
		levelThreeHeight.onTrue(new LevelThreeHeight().withTimeout(3));

		//Triggers
		commandController.leftTrigger(.1).whileTrue(new IntakeInOutCommand(IntakeInOutCommand.Direction.IN));
		commandController.rightTrigger(.1).whileTrue(new IntakeInOutCommand(IntakeInOutCommand.Direction.OUT));

		//Right Bumper
		JoystickButton setCube = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
		setCube.onTrue(new SetMode(Gamepiece.CUBE));

		//Left Bumper
		JoystickButton setCone = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
		setCone.onTrue(new SetMode(Gamepiece.CONE));

		POVButton doubleSub = new POVButton(operatorController, RobotMap.ControlParameters.DpadDirection.UP.value);
		doubleSub.onTrue(new DoubleSubHeight().withTimeout(3));

		POVButton StandingGamepiece = new POVButton(operatorController, RobotMap.ControlParameters.DpadDirection.LEFT.value);
		StandingGamepiece.onTrue(new FloorStandingHeight().withTimeout(3));

		POVButton tippedGamepiece = new POVButton(operatorController, RobotMap.ControlParameters.DpadDirection.RIGHT.value);
		tippedGamepiece.onTrue(new FloorTippedHeight().withTimeout(3));

		POVButton tipped2 = new POVButton(operatorController, RobotMap.ControlParameters.DpadDirection.DOWN.value);
		tipped2.onTrue(new FloorTippedHeight().withTimeout(3));

		JoystickButton stowPos = new JoystickButton(operatorController, XboxController.Button.kBack.value);
		stowPos.onTrue(new StowCommand().withTimeout(5));

		JoystickButton brakeModeButton = new JoystickButton(rightJoystick, 3);
		brakeModeButton.onTrue(new SetBrakeCommand());

		JoystickButton charge = new JoystickButton(leftJoystick, 5);
		charge.whileTrue(new StayLevelCommand());
	}
}
