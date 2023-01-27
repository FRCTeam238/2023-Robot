/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
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
public class CommandDescriptor {
    private String name;
    public String getName(){
        return name;
    }
    public void setName(String val){
        this.name = val;
    }

    private List<String> parameters = new ArrayList<>();
    public List<String> getParameters(){
        return parameters;
    }
    public void seteParameters(ArrayList<String> val){
        this.parameters = val;
    }

    private String isParallel;
    public String getParallelType(){
        return isParallel;
    }
    public void setParallelType(String val){
        this.isParallel = val;
    }
}
