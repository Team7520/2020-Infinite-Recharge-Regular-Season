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
// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
// import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSub extends SubsystemBase {

  private TalonSRX motor;
  //private DoubleSolenoid intakeDoubleSolenoid;
  private Solenoid intakeSingleSolenoid;

  /**
   *  IntakeSub has a motor, and two  {@link DoubleSolenoid}.
   *  if leftDoubleSolenoid and rightDoubleSolenoid share one controller, then only need one of 
   *  leftDoubleSolenoid and rightDoubleSolenoid.
   * 
   *  EDIT: Only one single solenoid is used for the intake drawer
   */
  //public IntakeSub(TalonSRX motor, DoubleSolenoid intakeDoubleSolenoid) {
    
  /**
   * Creates a new IntakeSub.
   */
  public IntakeSub(TalonSRX motor, Solenoid intakeSingleSolenoid) {
      this.motor = motor;
    motor.configOpenloopRamp(0.3);
    motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 0, 120, 1.5));

//    this.intakeDoubleSolenoid = intakeDoubleSolenoid;
    this.intakeSingleSolenoid = intakeSingleSolenoid;
} 

  public void  drawerSlideOut (){
//    intakeDoubleSolenoid.set(kForward);
    intakeSingleSolenoid.set(true);
  }

  public void  drawerSlideIn (){  
//    intakeDoubleSolenoid.set(kReverse);
    intakeSingleSolenoid.set(false);
  }

  public void rollingBallIn(double speed) {
    motor.set(ControlMode.PercentOutput, speed);
  }

  public void rollingBallOut(double speed) {
    motor.set(ControlMode.PercentOutput, -speed);
  }
  
  public void stopRolling() {
    motor.set(ControlMode.PercentOutput, 0);
  }
  
  public void stop(){
    motor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
