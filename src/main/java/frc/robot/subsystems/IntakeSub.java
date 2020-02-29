/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSub extends SubsystemBase {

  private VictorSPX motor;
  private DoubleSolenoid leftDoubleSolenoid;
  private DoubleSolenoid rightDoubleSolenoid;

  /**
   *  IntakeSub has a motor, and two  {@link DoubleSolenoid}.
   *  if leftDoubleSolenoid and rightDoubleSolenoid share one controller, then only need one of 
   *  leftDoubleSolenoid and rightDoubleSolenoid.
   */
  public IntakeSub(VictorSPX motor, DoubleSolenoid leftDoubleSolenoid, DoubleSolenoid rightDoubleSolenoid) {
    this.motor = motor;
    motor.configOpenloopRamp(1.0);
    //motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 0, 120, 1.5));

    this.leftDoubleSolenoid = leftDoubleSolenoid;
    this.rightDoubleSolenoid = rightDoubleSolenoid;
  } 

  public void  DrawerSlideOut (){
    leftDoubleSolenoid.set(kForward);
    rightDoubleSolenoid.set(kForward);
  }

  public void  DrawerSlideIn (){
    leftDoubleSolenoid.set(kReverse);
    rightDoubleSolenoid.set(kReverse);
  }

  public void rollingBallIn() {
    motor.set(ControlMode.PercentOutput, 0.5);
  }

  public void rollingBallOut() {
    motor.set(ControlMode.PercentOutput, -0.5);
  }
  
  public void stop(){
    motor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
