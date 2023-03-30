// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Arm extends SubsystemBase {

  private final boolean debug = true;
  
  private final TalonSRX armTalon = RobotMap.ArmParameters.armMotor;
  private final SingleJointedArmSim m_armSim = new SingleJointedArmSim(
    DCMotor.getVex775Pro(1), 374, 1.177, .3556, Units.degreesToRadians(-90), Units.degreesToRadians(155), true);
  private final TalonSRXSimCollection m_motorSim = armTalon.getSimCollection();

  protected StringLogEntry logCommand;
  protected DoubleLogEntry logEncoder;
  protected DoubleLogEntry logDesiredV;
  protected DoubleLogEntry logActualV;
  protected DoubleLogEntry logDesiredEncoder;
  protected DoubleLogEntry logFF;

  //TODO: Needs an ArmFeedforward. Estimates: kG = 1.24, kV = 2.29, kA = 0.07, kS = 0
  //TODO: Needs methods to get encoder value and encoder velocity

  /** Creates a new Arm. */
  public Arm() {
    logEncoder = new DoubleLogEntry(DataLogManager.getLog(), "Arm:Position");
    logDesiredV = new DoubleLogEntry(DataLogManager.getLog(), "Arm:Desired V");
    logActualV = new DoubleLogEntry(DataLogManager.getLog(), "Arm:Actual V");
    logDesiredEncoder = new DoubleLogEntry(DataLogManager.getLog(), "Arm:Desired Position");
    logFF = new DoubleLogEntry(DataLogManager.getLog(), "Arm:FF");
    logCommand = new StringLogEntry(DataLogManager.getLog(), "Arm:Command");
  }

  public void moveArmPercent (double armPercent, double armVelocity) {
    RobotMap.ArmParameters.armMotor.set(ControlMode.PercentOutput, armPercent);
  }

  //TODO: Should take a single input of type MotionProfile.State. This will have a position, velocity and acceleration
  // Use velocity and acceleration to calculate feedforward voltage using ArmFeedforward
  // Use position control mode on talon with value position and arbFeedForward as calculated
  public void moveArmVelocity() {

    if(SmartDashboard.getBoolean("ArmDebugging", debug))
    {
      SmartDashboard.putNumber("Arm Desired V", currentState.velocity);
      SmartDashboard.putNumber("Arm Desired Pos", currentState.position);
      SmartDashboard.putNumber("Arm Actual V", getVelocity());
      SmartDashboard.putNumber("Arm Actual Pos", getEncoderPosition());
      SmartDashboard.putNumber("Arm FF", feed);
    } else {
      logDesiredV.append(currentState.velocity);
      logDesiredEncoder.append(currentState.position);
      logActualV.append(getVelocity());
      logFF.append(feed);
    }
  }
  
  public void initControls() {
    // TODO set current limit
    // setup encoder
    // set PID values
    // set soft limits
  }

  @Override
  public void periodic()
  {
    logEncoder.append(getEncoder());
  }
  
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_armSim.setInput(armTalon.getMotorOutputVoltage() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(0.020);

    m_motorSim.addQuadraturePosition((int)(m_armSim.getAngleRads()*4096/(2*Math.PI)));
    SmartDashboard.putNumber("Sim Arm Angle", Units.radiansToDegrees(m_armSim.getAngleRads()));
    SmartDashboard.putNumber("Sim Arm Encoder", (int)(m_armSim.getAngleRads()*4096/(2*Math.PI)));

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));
  }

  public void putCommandString(Command command) {
    if(debug){
      SmartDashboard.putString("Arm Command", command.getName());
    } else {
      logCommand.append(command.getName());
    }
  }
}
