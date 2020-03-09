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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSub extends SubsystemBase {

  private TalonFX masterMotor;
  private TalonFX followerMotor; 

  private double minOutput;
  private double maxOutput;
  
  private double motorTargetRPM = 1000;
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

  public double getMotorTargetRPM() {
    return motorTargetRPM;
  }

  public void SetMotorTargetRPM(double rpm) {
    motorTargetRPM = rpm;
  }

  public double getMotorPosition() {
    // Falcon 500 Abs Position
    double motorPosition = masterMotor.getSensorCollection().getIntegratedSensorAbsolutePosition();

    return motorPosition;
  }
  
  public double getMotorRPM() {
    double magVel_UnitsPer100ms = masterMotor.getSelectedSensorVelocity(0);
		/**
		 * Convert to RPM
		 * https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
		 * MagRPM = magVel [units/kT] * 600 [kTs/minute] / 4096(units/rev), where kT = 100ms
		 */
		double motor_RPM = magVel_UnitsPer100ms * 600 / 4096;

    return motor_RPM;
  }

  public double getFlyWheelRPM() {
		double motor_RPM = getMotorRPM();

    double ratio = 2.2222;
    double flyWheel_RPM = motor_RPM * ratio; 

    return flyWheel_RPM;
  }


  public double getShootingSpeed() {
    // Shooting speed, that is linear velocity

    double flyWheel_RPM = getFlyWheelRPM();
    double flyWheelRadius = 3; // inches
    double shootingSpeedSpeed = flyWheel_RPM * 2 * flyWheelRadius * 3.14159 / 12; // feet

    return shootingSpeedSpeed;
  }
  
  public void showValues() {  
    SmartDashboard.putNumber("Shooter Motor Position", getMotorPosition());
    SmartDashboard.putNumber("Shooter Motor Target RPM", getMotorTargetRPM());
    SmartDashboard.putNumber("Shooter Motor RPM", getMotorRPM());
    SmartDashboard.putNumber("Shooter FlyWheel RPM", getFlyWheelRPM());
    SmartDashboard.putNumber("Shooting Speed (feet)", getShootingSpeed());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
