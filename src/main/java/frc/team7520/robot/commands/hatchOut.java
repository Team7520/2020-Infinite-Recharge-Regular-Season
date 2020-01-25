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

public class hatchOut extends Command {
  public double timeout = 0.25; //seconds
  private static hatchOut instance;

  public hatchOut() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_subHatch);

    setTimeout(timeout);
    setInterruptible(true);
  }

  public static hatchOut getInstance() {
    if(instance == null) {
      instance = new hatchOut();
    }
    return instance;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //Robot.m_subHatch.hatchOut();
    Robot.m_subHatch.setIsInUse(true);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_subHatch.hatchOut();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_subHatch.cancel();
    Robot.m_subHatch.setIsInUse(false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
