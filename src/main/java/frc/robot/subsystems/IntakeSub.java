/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSub extends SubsystemBase {

  private TalonSRX masterMotor;
  private TalonSRX followerMotor; 

  /**
   * Creates a new IntakeSub.
   */
  public IntakeSub(TalonSRX master, TalonSRX follower) {
    masterMotor = master;
    followerMotor = follower;
    followerMotor.set(ControlMode.Follower, masterMotor.getDeviceID());
    masterMotor.configOpenloopRamp(1.0);
  }

  public void forward() {
    masterMotor.set(ControlMode.PercentOutput, 1.0);
  }

  public void backward() {
    masterMotor.set(ControlMode.PercentOutput, -1.0);
  }
  
  public void stop(){
    masterMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
