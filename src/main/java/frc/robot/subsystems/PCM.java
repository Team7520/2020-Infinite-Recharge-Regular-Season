/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;

public class PCM extends SubsystemBase {
  private Compressor c;

  /**
   * Creates a new PCM of pneumatic system test.
   */
  public PCM() {

    c = new Compressor(0);

    c.setClosedLoopControl(true);
    c.setClosedLoopControl(false);
    boolean enabled = c.enabled();
    boolean pressureSwitch = c.getPressureSwitchValue();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // monitoring power status, usage, if necessary

  }

 
  

  /** 
   *  Reading the current
  */
  public double getCurrent() {
    return c.getCompressorCurrent();
  }

}
