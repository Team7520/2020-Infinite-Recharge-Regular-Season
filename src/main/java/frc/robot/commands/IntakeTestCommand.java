/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.IntakeSub;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class IntakeTestCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSub intake;
  private Joystick XboxControl;
  private JoystickButton slideIn; 
  private JoystickButton slideOut;

  /**
   * Creates a new IntakeTestCommand.
   *
   * @param subsystem The IntakeSub subsystem used by this command.
   * @param XboxControl The Joystick controlling the intake subsystem.
   * @param slideIn The button to retract the intake drawer.
   * @param slideOut The button to extend the intake drawer.
   */
  public IntakeTestCommand(IntakeSub subsystem, Joystick XboxControl, JoystickButton slideIn, JoystickButton slideOut) {
    intake = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.XboxControl = XboxControl;
    this.slideIn = slideIn;
    this.slideOut = slideOut;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftTrigger = XboxControl.getRawAxis(2);
    double rightTrigger = XboxControl.getRawAxis(3);
    if(Math.abs(leftTrigger) > 0 )
      intake.rollingBallIn(leftTrigger);
    else if(Math.abs(rightTrigger) > 0 )
      intake.rollingBallOut(rightTrigger);
    else
      intake.stopRolling();
    
    if(slideIn.get()){
      intake.drawerSlideIn();
    } else if(slideOut.get()){
      intake.drawerSlideOut();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
