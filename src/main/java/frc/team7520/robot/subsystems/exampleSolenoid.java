/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team7520.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import frc.team7520.robot.Robot;

/**
 * Add your docs here.
 */
public class exampleSolenoid extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

// public static Solenoid exampleSolenoid1 = new Solenoid(0);
 public static DoubleSolenoid exampleSolenoid1 = new DoubleSolenoid(0, 1);

  public static void pistonOut() {
    exampleSolenoid1.set(DoubleSolenoid.Value.kForward);

  }

  public static void pistonIn() {
    exampleSolenoid1.set(DoubleSolenoid.Value.kReverse);
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
