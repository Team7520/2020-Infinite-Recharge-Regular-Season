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


public class ArmToUpLimit extends Command {
  public boolean canceled = false;
  public double timeout = 1.0; //seconds
  public ArmToUpLimit() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_subArm);

    setTimeout(timeout);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_subArm.down();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(isTimedOut()){
      return true;
    } else {
      return false;
    } 
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_subArm.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
