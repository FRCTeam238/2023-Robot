/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.core238.wrappers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Button;
/**
 * A {@link Button} that gets its state from a {@link GenericHID}.
 */
public class TriggerButton extends Button {

  private final GenericHID m_joystick;
  private final int m_buttonNumber;

  /**
   * Create a joystick button for triggering commands.
   *
   * @param joystick     The GenericHID object that has the button (e.g. Joystick, KinectStick,
   *                     etc)
   * @param buttonNumber The button number (see {@link GenericHID#getRawButton(int) }
   */
  public TriggerButton(GenericHID joystick, int buttonNumber) {
    m_joystick = joystick;
    m_buttonNumber = buttonNumber;
  }

  /**
   * Gets the value of the joystick button.
   *
   * @return The value of the joystick button
   */
  @Override
  public boolean getAsBoolean() {
    boolean isTriggered = Math.abs(m_joystick.getRawAxis(m_buttonNumber)) >= 0.1;
    return isTriggered;
  }
}
