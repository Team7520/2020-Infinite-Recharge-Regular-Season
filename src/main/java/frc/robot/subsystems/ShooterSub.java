/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ShooterSub extends SubsystemBase {

  private TalonFX masterMotor;
  private TalonFX followerMotor; 

  private double minOutput;
  private double maxOutput;
  
  /**
   * Creates a new IntakeSub.
   */
  public ShooterSub(TalonFX master, TalonFX follower) {
    masterMotor = master;
    followerMotor = follower;

    followerMotor.set(ControlMode.Follower, masterMotor.getDeviceID());
    followerMotor.setInverted(TalonFXInvertType.OpposeMaster);
    // The slowest ramp possible is ten seconds (from neutral to full), though this is quite excessive.
    masterMotor.configOpenloopRamp(3.0);
    masterMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 0, 120, 1.5));
    masterMotor.configPeakOutputForward(0.40, 10);
    masterMotor.configPeakOutputReverse(-0.40, 10);
  }

  public void shoot(double speed) {
    masterMotor.set(ControlMode.PercentOutput, speed);
  }
  
  public void stop(){
    masterMotor.set(ControlMode.PercentOutput, 0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
