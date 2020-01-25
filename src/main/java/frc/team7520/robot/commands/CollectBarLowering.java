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

public class CollectBarLowering extends Command {
  public double timeout = 1.0; //seconds
  private static CollectBarLowering instance;

  public CollectBarLowering() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_subCollectBars);

    setTimeout(timeout);
    setInterruptible(true);
  }

  public static CollectBarLowering getInstance() {
    if(instance == null) {
      instance = new CollectBarLowering();
    }
    return instance;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_subCollectBars.setIsInUse(true);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_subCollectBars.lowerDown();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut();
    //return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_subCollectBars.stop();
    Robot.m_subCollectBars.setIsInUse(false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
