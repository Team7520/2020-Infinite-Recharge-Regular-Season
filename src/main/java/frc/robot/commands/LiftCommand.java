/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.LiftSub;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * A command to control the LiftSub subsystem.
 */
public class LiftCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LiftSub lift;
  private Joystick XboxControl;
  private JoystickButton up;
  private JoystickButton down;
  private JoystickButton open;
  private JoystickButton close;

  /**
   * Creates a new LiftCommand.
   *
   * @param lift The LiftSub subsystem used by this command.
   * @param XboxControl The Joystick controlling the lift subsystem.
   * @param up The button to loosen the lift winch.
   * @param down The button to tighten the lift winch.
   * @param open The button to bring the lift arm up.
   * @param down The button to bring the lift arm down.
   */
  public LiftCommand(LiftSub lift, Joystick XboxControl, JoystickButton up, JoystickButton down, JoystickButton open, JoystickButton close) {
    this.lift = lift;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lift);
    this.XboxControl = XboxControl;
    this.up = up;
    this.down = down;
    this.open = open;
    this.close = close;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(open.get()){
        lift.armOpen();
    } else if(close.get()){
        lift.armClose();
    }

    if(up.get()){
        lift.winchLoosen();
    } else if(down.get()){
        lift.winchTighten();
    } else {
        lift.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lift.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
