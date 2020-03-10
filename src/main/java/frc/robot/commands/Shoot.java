/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Shoot extends CommandBase {

  private ShooterSub m_shooterSub;

  private Joystick joystick;

  /**
   * Creates a new Shooter.
   */
  public Shoot(ShooterSub subsystem, Joystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterSub = subsystem;
    addRequirements(m_shooterSub);

    this.joystick = joystick;
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
    m_shooterSub.shoot(joystick.getRawAxis(5));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSub.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
