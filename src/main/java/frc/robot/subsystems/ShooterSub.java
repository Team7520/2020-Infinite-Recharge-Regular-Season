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

  private double m_newMotorOutput;
  private double m_lastMotorOutput;
  
  private double motorTargetRPM = 1000;
  private double diffToleranceRPM = 100; // different from target RPM in reange [ -100, +100] 
  private boolean m_inAutoCommand = false;
  private double m_distance = 10.0;

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

    m_newMotorOutput = 0;
    m_lastMotorOutput = 0;
  }

  public void shoot(double speed) {
    motor1.set(ControlMode.PercentOutput, speed);
    motor2.set(ControlMode.PercentOutput, -speed);
    m_lastMotorOutput = speed;
  }

  public void stop(){
    motor1.set(ControlMode.PercentOutput, 0);
    motor2.set(ControlMode.PercentOutput, 0);
  }

  public double getMotorTargetRPM() {
    return motorTargetRPM;
  }

  public void SetTargetDistance(double distance) {
    m_distance = distance;

    // To do, find out correct match of RPM and distance
    motorTargetRPM = m_distance * 20;
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

  public boolean lowerThanMotorTargetRPM() {
    double motorRPM = getMotorRPM();

    if (motorRPM < motorTargetRPM - diffToleranceRPM)
       return true;
       
    return false;
        
  }

  public boolean higherThanMotorTargetRPM() {
    double motorRPM = getMotorRPM();

    if (motorRPM > motorTargetRPM + diffToleranceRPM)
       return true;
       
    return false;
        
  }

  public boolean reachedMotorTargetRPM() {
    // it will call getMotorRPM() 2 times and get 2 different values
    // if (!lowerThanMotorTargetRPM() && !higherThanMotorTargetRPM())
    //      return true;

    double motorRPM = getMotorRPM();

    if (motorRPM >= motorTargetRPM - diffToleranceRPM &&
        motorRPM <= motorTargetRPM + diffToleranceRPM)
       return true;
    
    return false;

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
    double shootingSpeed = flyWheel_RPM * 2 * flyWheelRadius * Math.PI / 12; // feet

    return shootingSpeed;
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

  public void setInAutoCommand(boolean inAutoCommand) {
    m_inAutoCommand = inAutoCommand;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!m_inAutoCommand)
      return;

    // stage 1: speed up
    // stage 2: keep target RPM and shoot 
    // stage 3: slow down 
    
    double alphaOutput = 1.0 / (3 * 50); // 3 seconds, 50 loops per second
    if(reachedMotorTargetRPM())
      m_newMotorOutput = m_lastMotorOutput;
    else if (lowerThanMotorTargetRPM())
      m_newMotorOutput += alphaOutput;
    else if (higherThanMotorTargetRPM())
      m_newMotorOutput -= alphaOutput;

    if (m_newMotorOutput > 1.0)
      m_newMotorOutput = 1.0;

    if (m_newMotorOutput < 0)
      m_newMotorOutput = 0;
      
    shoot(m_newMotorOutput); 
  }
}
