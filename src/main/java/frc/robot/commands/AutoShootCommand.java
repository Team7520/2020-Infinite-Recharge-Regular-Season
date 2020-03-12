/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

public class AutoShootCommand extends CommandBase {

  private ShooterSub m_shooterSub;
  private FeederSub m_feederSub;
  private double m_speed = 0.1;
  private double m_distance = 10; // feet // start (white) line

  /**
   * Creates a new Shooter.
   */
  public AutoShootCommand(ShooterSub shooterSub, FeederSub feederSub, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterSub = shooterSub;
    m_feederSub = feederSub;
    addRequirements(m_shooterSub);
    addRequirements(m_feederSub);

    m_distance = distance;

  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // stop the motor
    // init flags, such as RPM reached desired value
    m_shooterSub.SetTargetDistance(m_distance);
    m_shooterSub.setInAutoCommand(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // To do: check status
    m_shooterSub.shoot(m_speed);
    if (m_shooterSub.reachedMotorTargetRPM())
      m_feederSub.forwards();
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
