// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
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
import frc.core238.MotionProfile;
import frc.robot.Robot;
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
  protected ArmFeedforward ff;

  public boolean PIDEnabled = true;

  /** Creates a new Arm. */
  public Arm() {
    logEncoder = new DoubleLogEntry(DataLogManager.getLog(), "Arm:Position");
    logDesiredV = new DoubleLogEntry(DataLogManager.getLog(), "Arm:Desired V");
    logActualV = new DoubleLogEntry(DataLogManager.getLog(), "Arm:Actual V");
    logDesiredEncoder = new DoubleLogEntry(DataLogManager.getLog(), "Arm:Desired Position");
    logFF = new DoubleLogEntry(DataLogManager.getLog(), "Arm:FF");
    logCommand = new StringLogEntry(DataLogManager.getLog(), "Arm:Command");
    ff = new ArmFeedforward(RobotMap.ArmParameters.kS, RobotMap.ArmParameters.kG, RobotMap.ArmParameters.kV, RobotMap.ArmParameters.kA);
    initControls();
  }

  public void moveArmPercent (double armPercent) {
    if(!PIDEnabled)
    {
      armPercent = armPercent *1.5; //needs exra power if no feedforward to "float it"
    }
    double feed = ff.calculate(Units.degreesToRadians(getEncoderPosition()), 0)/12;
    RobotMap.ArmParameters.armMotor.set(ControlMode.PercentOutput, armPercent + feed);
    if(SmartDashboard.getBoolean("ArmDebugging", debug))
    {
      SmartDashboard.putNumber("Arm FF", feed*12);
    } else {
      logFF.append(feed*12);
    }
  }

  // Use velocity and acceleration to calculate feedforward voltage using ArmFeedforward
  // Use position control mode on talon with value position (convert to 4096 units) and arbFeedForward as calculated
  public void moveArmVelocity(MotionProfile.State currentState) {
    if(!PIDEnabled)
    {
      return;
    }
    double position = Units.degreesToRadians(currentState.position);
    double acceleration = Units.degreesToRadians(currentState.acceleration);
    double velocity = Units.degreesToRadians(currentState.velocity);
    double feed = ff.calculate(position, velocity, acceleration);

    
    
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
    feed = Math.min(Math.max(feed,-RobotMap.ArmParameters.voltageLimit),RobotMap.ArmParameters.voltageLimit);
    SmartDashboard.putNumber("Arm FF Limited", feed);
    armTalon.set(ControlMode.Position, position * 4096.0 / (2.0*Math.PI), DemandType.ArbitraryFeedForward, feed / RobotMap.DrivetrainParameters.maxVoltage);
  }
  
  public void holdPosition(double position)
  {
    if(!PIDEnabled)
    {
      return; //Can't hold position without encoder
    }
    double feed = ff.calculate(Units.degreesToRadians(position), 0);
    armTalon.set(ControlMode.Position, position * 4096.0 / (2.0*Math.PI), DemandType.ArbitraryFeedForward, feed / RobotMap.DrivetrainParameters.maxVoltage);
    if(SmartDashboard.getBoolean("ArmDebugging", debug))
    {
      SmartDashboard.putNumber("Arm FF", feed);
      SmartDashboard.putNumber("Arm Actual Pos", getEncoderPosition());
      SmartDashboard.putNumber("Arm Desired Pos", position);
    } else {
      logDesiredEncoder.append(position);
      logFF.append(feed);
    }
  }

  public double getVelocity() {
    return armTalon.getSelectedSensorVelocity() * 10.0 * (360.0/4096.0);
  }

  public double getEncoderPosition() {
    return armTalon.getSelectedSensorPosition() * (360.0/4096.0);
  }
  public void initControls() {
    armTalon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 40, 100));
    armTalon.enableCurrentLimit(true);
    //setup encoder
    armTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    int absPos = armTalon.getSensorCollection().getPulseWidthPosition();
    //TODO: Is this what it returns if no sensor?
    if(absPos == 0)
    {
      PIDEnabled = false;
      return;
    }
    int relPos;
    System.out.println("Absolute Position: " + absPos);
    if (absPos > 1600) {
      relPos = RobotMap.ArmParameters.armOffset - absPos;
    } else {
      relPos = RobotMap.ArmParameters.armOffset - 4096 - absPos;
    }
    System.out.println("Relative Position: " + relPos);
    if(Robot.isReal())
    {
      armTalon.setSelectedSensorPosition(-relPos);
    }

    armTalon.setSensorPhase(false);

    armTalon.setInverted(true);
    armTalon.config_kP(0, RobotMap.ArmParameters.kP);
    armTalon.config_kI(0 ,RobotMap.ArmParameters.kI);
    armTalon.config_kD(0, RobotMap.ArmParameters.kD);


    armTalon.configForwardSoftLimitThreshold(RobotMap.ArmParameters.armUpperLimit);
    armTalon.configReverseSoftLimitThreshold(RobotMap.ArmParameters.armLowerLimit);
    armTalon.configForwardSoftLimitEnable(true);
    armTalon.configReverseSoftLimitEnable(true);
  }

  @Override
  public void periodic()
  {
    logEncoder.append(getEncoderPosition());
  }
  
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_armSim.setInput(armTalon.getMotorOutputVoltage() * RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("ArmOutput", armTalon.getMotorOutputPercent());
    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(0.020);

    m_motorSim.setQuadratureRawPosition((int)(-m_armSim.getAngleRads()*4096/(2*Math.PI)));
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

  public boolean hitsBumper() {
   return getEncoderPosition() <= RobotMap.ArmParameters.tippedConeFloor;
  }

}
