/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class PDP extends SubsystemBase {
    PowerDistributionPanel PDP;
  /**
   * Creates a new ExampleSubsystem.
   */
  public PDP() {
    // Warning: To enable voltage and current logging in the Driver Station, the CAN ID for
    // the PDP must be 0.
    PDP = new PowerDistributionPanel(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // monitoring power status, usage, if necessary

  }

  /** Monitoring the bus voltage can be useful for (among other things) detecting 
   *  when the robot is near a brownout, so that action can be taken to avoid brownout 
   *  in a controlled manner.
  */
  public double getVoltage() {
    // This method will be called once per scheduler run
    // monitoring power status, usage, if necessary
    return PDP.getVoltage(); 
  }
  
  /** 
   * Reading the Temperature
   * Monitoring the temperature can be useful for detecting if the robot has been drawing too
   * much power and needs to be shut down for a while, or if there is a short or other wiring
   * problem
  */
  public double getTemperature() {
    return PDP.getTemperature(); 
  }


  /** 
   *  Reading the Total Current
  */
  public double getTotalCurrent() {
    return PDP.getTotalCurrent(); 
  }

  /** 
   *  Reading the Total Energy
  */
  public double getTotalEnergy() {
    return PDP.getTotalEnergy(); 
  }

  /** 
   *  Reading the Individual Channel Currents
  */
  public double getCurrent(int channel) {
    return PDP.getCurrent(channel); 
  }

}
