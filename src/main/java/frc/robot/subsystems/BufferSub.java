/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

public class BufferSub extends SubsystemBase {
  
//  private VictorSPX buffer;
  private TalonFX buffer;
  
  /**
   * Creates a new BufferSub.
   */
//  public BufferSub(VictorSPX buffer) {
    public BufferSub(TalonFX buffer) {
      this.buffer = buffer;
  }

  public void forwards(){
    buffer.set(ControlMode.PercentOutput, 1.0);
  }

  public void backwards(){
    buffer.set(ControlMode.PercentOutput, -1.0);
  }

  public void stop(){
    buffer.set(ControlMode.PercentOutput, 0);
  }

  public boolean direction(boolean forwards){
    return forwards;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
