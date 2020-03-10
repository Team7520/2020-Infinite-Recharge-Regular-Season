/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.IntakeSub;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class AutoIntakeCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSub intake;
  public double m_timeout = 2;
  private double m_speed;  
  private final Timer m_timer = new Timer();
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoIntakeCommand(IntakeSub subsystem, double speed, double timeout) {
    intake = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    m_speed = speed;
    m_timeout = timeout;

    m_timer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.drawerSlideOut();
    intake.rollingBallIn(m_speed);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_timer.hasElapsed(m_timeout))
      end(false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopRolling();
    intake.drawerSlideIn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    intake.stopRolling();
    intake.drawerSlideIn();
    return false;
  }
}
