/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team7520.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.team7520.robot.Robot;
/**
 * Add your docs here.
 */
public class CollectBarsSub extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private VictorSPX motor;
  public DigitalInput upLimitSwitch; 

  private double speed = 1.0;
  // current usage (by a command) status 
  public boolean isUsed = false;
   // the running command using this subsystem will be canceled with this boolean
  public boolean canceled = false;

  public void setMotor(VictorSPX masterCollectMotor) {
    motor = masterCollectMotor;
  }

  public void setUpLimitSwitch(DigitalInput limitSwitch) {
    upLimitSwitch = limitSwitch;
  }

  public boolean hitUpLimitSwitch() {
    return upLimitSwitch.get();
  }

  public void raiseUp() {
    motor.set(ControlMode.PercentOutput, speed);
  }

  public void lowerDown() {
    motor.set(ControlMode.PercentOutput, -speed);
  }

 //public static void stop() {
  public void stop() {
    motor.set(ControlMode.PercentOutput, 0);
  }

  public void setIsInUse(boolean status) {
    isUsed = status;
}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
