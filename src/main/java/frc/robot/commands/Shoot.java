/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Shoot extends CommandBase {

  private IntakeSub m_intakeSub;

  /**
   * Creates a new Shooter.
   */
  public Shoot(IntakeSub subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intakeSub = subsystem;
    addRequirements(m_intakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // stop the motor
    // init flags, such as RPM reached desired value
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // To do: check status
    m_intakeSub.forward();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSub.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
