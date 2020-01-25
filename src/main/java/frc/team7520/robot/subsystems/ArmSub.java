/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team7520.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.team7520.robot.Robot;
/**
 * Add your docs here.
 */
public class ArmSub extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private VictorSPX motor;
  private double armSpeed = 0.7;

  public void setMotor(VictorSPX armMotor) {
    motor = armMotor;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void up() {
    motor.set(ControlMode.PercentOutput, armSpeed);
  }

  public void down() {
    motor.set(ControlMode.PercentOutput, -armSpeed);
  }

//public static void hatchStop() {
  public void stop() {
    motor.set(ControlMode.PercentOutput, 0);
  }

}
