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

  private TalonFX motor1;
  private TalonFX motor2; 

  private double minOutput;
  private double maxOutput;
  
  private double motorTargetRPM = 1000;
  /**
   * Creates a new ShooterSub.
   */
  public ShooterSub(TalonFX motor1, TalonFX motor2) {
    this.motor1 = motor1;
    this.motor2 = motor2;

    // The slowest ramp possible is ten seconds (from neutral to full), though this is quite excessive.
    motor1.configOpenloopRamp(3.0);
    motor1.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 0, 120, 1.5));
    motor1.configPeakOutputForward(0.40, 10);
    motor1.configPeakOutputReverse(-0.40, 10);

    motor2.configOpenloopRamp(3.0);
    motor2.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 0, 120, 1.5));
    motor2.configPeakOutputForward(0.40, 10);
    motor2.configPeakOutputReverse(-0.40, 10);
  }

  public void shoot(double speed) {
    motor1.set(ControlMode.PercentOutput, speed);
    motor2.set(ControlMode.PercentOutput, -speed);
  }
  
  public void stop(){
    motor1.set(ControlMode.PercentOutput, 0);
    motor2.set(ControlMode.PercentOutput, 0);
  }

  public double getMotorTargetRPM() {
    return motorTargetRPM;
  }

  public void SetMotorTargetRPM(double rpm) {
    motorTargetRPM = rpm;
  }

  public double getMotor1Position() {
    // Falcon 500 Abs Position
    double motorPosition = motor1.getSensorCollection().getIntegratedSensorAbsolutePosition();
    return motorPosition;
  }

  public double getMotor2Position() {
    // Falcon 500 Abs Position
    double motorPosition = motor2.getSensorCollection().getIntegratedSensorAbsolutePosition();
    return motorPosition;
  }
  
  public double getMotor1RPM() {
    double magVel_UnitsPer100ms = motor1.getSelectedSensorVelocity(0);
		/**
		 * Convert to RPM
		 * https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
		 * MagRPM = magVel [units/kT] * 600 [kTs/minute] / 4096(units/rev), where kT = 100ms
		 */
		double motor_RPM = magVel_UnitsPer100ms * 600 / 4096;

    return motor_RPM;
  }

  public double getMotor2RPM() {
    double magVel_UnitsPer100ms = motor2.getSelectedSensorVelocity(0);
		/**
		 * Convert to RPM
		 * https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
		 * MagRPM = magVel [units/kT] * 600 [kTs/minute] / 4096(units/rev), where kT = 100ms
		 */
		double motor_RPM = magVel_UnitsPer100ms * 600 / 4096;

    return motor_RPM;
  }

  public double getFlyWheelRPM() {
		double motor_RPM = (getMotor1RPM() + getMotor2RPM()) / 2;

    double ratio = 20/9;
    double flyWheel_RPM = motor_RPM * ratio; 

    return flyWheel_RPM;
  }


  public double getShootingSpeed() {
    // Shooting speed, that is linear velocity

    double flyWheel_RPM = getFlyWheelRPM();
    double flyWheelRadius = 3; // inches
    double shootingSpeedSpeed = flyWheel_RPM * 2 * flyWheelRadius * Math.PI / 12; // feet

    return shootingSpeedSpeed;
  }
  
  public void showValues() {  
    SmartDashboard.putNumber("Shooter Falcon 1 Position", getMotor1Position());
    SmartDashboard.putNumber("Shooter Falcon 2 Position", getMotor2Position());
    SmartDashboard.putNumber("Shooter Motor Target RPM", getMotorTargetRPM());
    SmartDashboard.putNumber("Shooter Falcon 1 RPM", getMotor1RPM());
    SmartDashboard.putNumber("Shooter Falcon 2 RPM", getMotor2RPM());
    SmartDashboard.putNumber("Shooter FlyWheel RPM", getFlyWheelRPM());
    SmartDashboard.putNumber("Shooting Speed (feet)", getShootingSpeed());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
