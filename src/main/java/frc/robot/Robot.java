// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.core238.DriverControls.driveType;
import frc.core238.autonomous.AutonomousModesReader;
import frc.core238.autonomous.DataFileAutonomousModeDataSource;
import frc.core238.autonomous.IAutonomousModeDataSource;
import frc.robot.subsystems.*;
import frc.robot.RobotMap.IntakeParameters.Gamepiece;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot { 
  private Command m_autonomousCommand; 
  public static Drivetrain drivetrain;
  public static Elevator elevator;
  public static Intake intake;
  public UsbCamera intakeCamera;
  public static Arm arm;
  OI oi;
  LEDs leDs;
  String lastSelectedAuto;
  HashMap<String, Command> m_autoModes;
  List<String> autoModeNames;
  SendableChooser<String> m_chooser = new SendableChooser<>();
  boolean fmsConnected = false;
  boolean m_allowAuto = true;
  AutonomousModesReader reader;
  public static Gamepiece gamepiece = Gamepiece.CUBE;
  Mechanism2d mechanism;
  MechanismRoot2d root;
  MechanismLigament2d elevator2D, carriage2D, arm2D, wrist2D;
  StringLogEntry gamepieceLog;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    mechanism = new Mechanism2d(50, 40);
    root = mechanism.getRoot("Mechanism", 5, 0);
    elevator2D = new MechanismLigament2d("Elevator", 5, 55);
    carriage2D = new MechanismLigament2d("Carriage", 24, -55);
    arm2D = new MechanismLigament2d("Arm", 15, 150);
    wrist2D = new MechanismLigament2d("Wrist", 14, -150);
    arm2D.append(wrist2D);
    carriage2D.append(arm2D);
    elevator2D.append(carriage2D);
    root.append(elevator2D);
    SmartDashboard.putData("Mechanism", mechanism);
    drivetrain = new Drivetrain();
    elevator = new Elevator();
    intake = new Intake();
    arm = new Arm();
    oi = new OI(driveType.Tank);
    if (isReal()) {
      intakeCamera = CameraServer.startAutomaticCapture();
      intakeCamera.setResolution(160, 120);
      intakeCamera.setFPS(20);
      leDs = new LEDs();
    }

    SmartDashboard.putBoolean("Autos Ready", true);

    new Trigger(this::isEnabled)
    .negate()
    .debounce(5)
    .onTrue(Commands.runOnce(drivetrain::setCoast, drivetrain).ignoringDisable(true));


    if(isSimulation())
    {
      setDeployDirectory();
    }

    // initialize the automodes list
    IAutonomousModeDataSource autoModesDataSource = new DataFileAutonomousModeDataSource(Filesystem.getDeployDirectory() + "/amode238.txt");
    reader = new AutonomousModesReader(autoModesDataSource);
    autoModeNames = reader.GetAutoNames();
    for (String name : autoModeNames) {
      m_chooser.setDefaultOption(name, name);
      System.out.println("ADDED" + name);
    }
    SmartDashboard.putData("Auto Modes", m_chooser);

    lastSelectedAuto = m_chooser.getSelected();
    
    m_autonomousCommand = reader.getAutonomousMode(lastSelectedAuto);

    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    elevator2D.setLength(5+Units.metersToInches(elevator.getPositionMeters()));
    arm2D.setAngle(arm.getEncoderPosition());
    wrist2D.setAngle(-arm.getEncoderPosition());
    SmartDashboard.putBoolean("GamepieceIsCube", gamepiece == Gamepiece.CUBE);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    drivetrain.setBrake();
  }

  @Override
  public void disabledPeriodic() {
    if (lastSelectedAuto != m_chooser.getSelected()) {
      SmartDashboard.putBoolean("Autos Ready", false);
      NetworkTableInstance.getDefault().flush();
      m_autonomousCommand = reader.getAutonomousMode(m_chooser.getSelected());
      SmartDashboard.putBoolean("Autos Ready", true);

      lastSelectedAuto = m_chooser.getSelected();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {

    drivetrain.setBrake();

    if (lastSelectedAuto != m_chooser.getSelected() || m_autonomousCommand.equals(null)) {
      m_autonomousCommand = reader.getAutonomousMode(m_chooser.getSelected());
      lastSelectedAuto = m_chooser.getSelected();
    }
    String autoMode = m_chooser.getSelected();

    if(m_allowAuto) {
      // schedule the autonomous command (example)
      m_autonomousCommand.schedule();
    }

    // prevent the robot from rerunning auto mode a second time without a restart
    if (DriverStation.isFMSAttached()) {
      m_allowAuto = false;

    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    //
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    drivetrain.setCoast();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    drivetrain.setBrake();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  public void setDeployDirectory() {
    File deployDir = Filesystem.getDeployDirectory();

    // remove everything starting at /build
    String userDir = deployDir.getPath();
    int idx = userDir.indexOf(File.separator + "build");
    if (idx > 0) {
        userDir = userDir.substring(0, idx);
        System.setProperty("user.dir", userDir);
    }
  }

  public static boolean isCube()
  {
    return Robot.gamepiece == RobotMap.IntakeParameters.Gamepiece.CUBE;
  }

}
