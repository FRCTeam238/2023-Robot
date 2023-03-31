// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.IntakeParameters.Gamepiece;

public class LEDs extends SubsystemBase {
  /** Creates a new LEDs. */
  AddressableLED ledStrip;
  AddressableLEDBuffer buffer;
  Gamepiece gamepiece;

  public LEDs() {
    ledStrip = new AddressableLED(9);
    buffer = new AddressableLEDBuffer(43);
    ledStrip.setLength(43);
  }

  @Override
  public void periodic() {
    //TODO: check if gamepiece stored in this class matches robot setting. If not, set the LED color to match the gamepiece
    //and update the storage in here. Don't want to just blindly set every cycle because it's a little time intensive
  }

  public void setColor(Color color)
  {
    for(int i=0; i<buffer.getLength(); i++)
    {
      buffer.setLED(i, color);
    }
    ledStrip.setData(buffer);
  }
}
