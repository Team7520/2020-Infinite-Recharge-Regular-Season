/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team7520.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team7520.robot.Robot;
import frc.team7520.robot.subsystems.*;

public class AutonSideCargoPanel extends Command {

  private static AutonSideCargoPanel instance;

  public AutonSideCargoPanel() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_driveTrain);

    setTimeout(3.0);
    setInterruptible(true);
  }

  public static AutonSideCargoPanel getInstance() {
    if(instance == null) {
      instance = new AutonSideCargoPanel();
    }
    return instance;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_driveTrain.isInAuto = true;
    Robot.m_driveTrain.goForwardGyro(12, 0, 0.5);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // if is canceled, then stop and end
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_driveTrain.stop();
    Robot.m_driveTrain.isInAuto = false;

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
