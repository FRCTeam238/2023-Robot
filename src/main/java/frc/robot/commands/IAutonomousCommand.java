/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;

public interface IAutonomousCommand {
    public boolean getIsAutonomousMode();
    public void setIsAutonomousMode(boolean isAutonomousMode);
    public void setParameters(List<String> parameters); 
    public double getTimeout();
}
