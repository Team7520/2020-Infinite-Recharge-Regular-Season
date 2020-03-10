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

import frc.robot.utilities.SpeedPlan;

public class Falcon500Sub extends SubsystemBase {
  
  private TalonFX falcon;
  private double max_abs_forward = 1.0;
  private double max_abs_backward = 1.0;

  private double distance = 0; 
  private double current_speed = 0;

  public SpeedPlan lightDutySpeedPlan = new SpeedPlan();

  /**
   * Creates a new BufferSub.
   */
    public Falcon500Sub(TalonFX falcon) {
      this.falcon = falcon;
      this.falcon.configFactoryDefault();

      lightDutySpeedPlan.init(0.2, max_abs_forward, 0.15, 40, 10);

  }

  public void init(){
    distance = 0;
    current_speed = 0;
  }

  public void move_to(double distance){
    falcon.set(ControlMode.PercentOutput, max_abs_forward);
  }

  // 
  public void forwards(){
    falcon.set(ControlMode.PercentOutput, max_abs_forward);
  }

  public void backwards(){
    falcon.set(ControlMode.PercentOutput, -max_abs_backward) ;
  }

  public void stop(){
    falcon.set(ControlMode.PercentOutput, 0);
  }

  public boolean direction(boolean forwards){
    return forwards;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // To do check current, speed, etc
  }
}
