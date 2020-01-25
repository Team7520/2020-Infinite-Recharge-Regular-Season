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
public class ConveyerSub extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private VictorSPX motor;
  private double speed = 0.75;

  public void setMotor(VictorSPX conveyerMotor) {
    motor = conveyerMotor;
  }

  public void moveCargoUp() {
    motor.set(ControlMode.PercentOutput, speed);
  }

  public void moveCargoDown() {
    motor.set(ControlMode.PercentOutput, -speed);
  }

//public static void stop() {
  public void stop() {
    motor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
