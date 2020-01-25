/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team7520.robot.sensors;


import edu.wpi.first.wpilibj.DigitalInput;
/**
 * Add your docs here.
 */
public class LimitSensors {
  
  // the following (touch) limit sensors are not connected to a
  // corresponding encoder.  
  // all limit switch sensor readings are reversed
  private DigitalInput armUpperLimitSwitch;
  private DigitalInput collectBarUpperLimitSwitch;
  private DigitalInput intakeLowerLimitSwitch;
 // private DigitalInput intakeUpperLimitSwitch;

  public void init() {
    collectBarUpperLimitSwitch = new DigitalInput(1);
    armUpperLimitSwitch = new DigitalInput(2);
    intakeLowerLimitSwitch = new DigitalInput(3);
    //intakeUpperLimitSwitch = new DigitalInput(4);
   
  }

  public boolean isPressedCollectBarUpperLimit()
  {
      // reading is reversed
      return !collectBarUpperLimitSwitch.get();
  }

  public boolean isPressedArmUpperLimit()
  {
      // reading is reversed
      return !armUpperLimitSwitch.get();
  }

  public boolean isPressedIntakeLowerLimit()
  {
      // reading is reversed
      return !intakeLowerLimitSwitch.get();
  }

//  public boolean isPressedIntakeUpperLimit()
//  {
      // reading is reversed
//      return !intakeUpperLimitSwitch.get();
//  }
}
