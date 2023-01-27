/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.core238.autonomous;

import java.util.ArrayList;
import java.util.List;

/**
 * Add your docs here.
 */
public class AutonomousModeDescriptors {
    private List<AutonomousModeDescriptor> AutonomousModes = new ArrayList<AutonomousModeDescriptor>();
    public List<AutonomousModeDescriptor> getAutonomousModes(){
        return AutonomousModes;
    }
    public void setAutonomousModes(ArrayList<AutonomousModeDescriptor> val){
        AutonomousModes = val;
    }
}
