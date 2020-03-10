/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj.Joystick;

public class FeedCommand extends CommandBase {

  private FeederSub feeder;
  private JoystickButton forwards;
  private JoystickButton backwards;

  /**
   * Creates a new Buffer.
   */
  public FeedCommand(FeederSub buffer, JoystickButton forwards, JoystickButton backwards) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.feeder = buffer;
    addRequirements(this.feeder);
    this.forwards = forwards;
    this.backwards = backwards;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(forwards.get()){
      feeder.forwards();
    } else if(backwards.get()){
      feeder.backwards();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
