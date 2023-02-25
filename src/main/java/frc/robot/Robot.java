// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.HashMap;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.core238.DriverControls.driveType;
import frc.core238.autonomous.AutonomousModesReader;
import frc.core238.autonomous.DataFileAutonomousModeDataSource;
import frc.core238.autonomous.IAutonomousModeDataSource;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

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
  OI oi;
  HashMap<String, Command> m_autoModes;
  SendableChooser<String> m_chooser = new SendableChooser<>();
  boolean fmsConnected = false;
  boolean m_allowAuto = true;


  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    drivetrain = new Drivetrain();
    elevator = new Elevator();
    intake = new Intake();
    oi = new OI(driveType.Tank);

    if(isSimulation())
    {
      setDeployDirectory();
    }

    // initialize the automodes list
    IAutonomousModeDataSource autoModesDataSource = new DataFileAutonomousModeDataSource(Filesystem.getDeployDirectory() + "/amode238.txt");
    AutonomousModesReader reader = new AutonomousModesReader(autoModesDataSource);
    m_autoModes = reader.getAutonmousModes();
  
      if (m_autoModes.size() > 0) {
        Boolean first = true;
  
        for (var entry : m_autoModes.entrySet()) {
          String name = entry.getKey();
          if (first) {
            first = false;
            m_chooser.setDefaultOption(name, name);
          } else {
            m_chooser.addOption(name, name);
          }
        }
        SmartDashboard.putData("Auto Modes", m_chooser);
      }
    }
    {
    DataLogManager.start();
    Shuffleboard.getTab("Logging").add(CommandScheduler.getInstance());
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
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {

    String autoMode = m_chooser.getSelected();

    // schedule the autonomous command (example)
    if (m_allowAuto && m_autoModes.containsKey(autoMode)) {
      m_autonomousCommand = m_autoModes.get(autoMode);
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
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
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

}
