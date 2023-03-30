// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

public class Arm extends SubsystemBase {

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Arm extends SubsystemBase {
  
  private final TalonSRX armTalon = new RobotMap.armParameters.armMotor;
  private final SingleJointedArmSim m_armSim = new SingleJointedArmSim(
    DCMotor.getVex775Pro(1), 200, 1.177, .3556, Units.degreesToRadians(-90), Units.degreesToRadians(155), true);
  private final TalonSRXSimCollection m_motorSim = armTalon.getSimCollection();


  /** Creates a new Arm. */
  public Arm() {
  
  }

  public void moveArmPercent (double armPercent, double armVelocity) {
    RobotMap.ArmParameters.armMotor.set(ControlMode.PercentOutput, armPercent);
  }

  public void moveArmVelocity() {
  }
  
  public void initControls() {
    // set current limit
    // setup encoder
    // set PID values
    // set soft limits
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
