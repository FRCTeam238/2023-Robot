// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {

  protected DoubleLogEntry logIntake;
  protected StringLogEntry logCommand;
  protected int stallCounter = 0;

  /** Creates a new Intake. */
  public Intake() {
    logCommand = new StringLogEntry(DataLogManager.getLog(), "Intake:Command");
    logIntake = new DoubleLogEntry(DataLogManager.getLog(), "Intake:Speed");

    RobotMap.IntakeParameters.intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
      true, RobotMap.IntakeParameters.continuousCurrent, 
      RobotMap.IntakeParameters.peakCurrent, RobotMap.IntakeParameters.peakDuration));
  }


  public void run (double intakeSpeed) {
    RobotMap.IntakeParameters.intakeMotor.set(ControlMode.PercentOutput, intakeSpeed);
    logIntake.append(intakeSpeed);
  }

  public void stop () {
    RobotMap.IntakeParameters.intakeMotor.set(ControlMode.PercentOutput, 0);
    logIntake.append(0);
  }

  public void putCommandString(Command command) {
      logCommand.append(command.getName());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean isStalling() {
    if (RobotMap.IntakeParameters.intakeMotor.getSupplyCurrent() > RobotMap.IntakeParameters.stallCurrent)
    {
      return true;
      //stallCounter++;
    } else {
      //stallCounter = 0;
      return false;
    }
    /*if(stallCounter > 3)
    {
      return true;
    } else {
      return false;
    }*/
  }
}
