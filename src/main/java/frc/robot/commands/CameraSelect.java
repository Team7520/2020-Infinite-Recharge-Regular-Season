/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

public class CameraSelect extends CommandBase {
  /**
   * Creates a new VisionActivate.
   */
  private String[] modes = { "intake", "shooter", "arm"};
  private Timer responseTimer;
  private final double delayTime = 0.5;

  public CameraSelect() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      
    responseTimer = new Timer();
    responseTimer.start();
      //SmartDashboard.putNumber("Distance", SmartDashboard.getNumberArray("vision/target_info", new double[]{0,0,0,0,0,0,0})[3]);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(responseTimer.hasElapsed(delayTime)){
        var mode = SmartDashboard.getString("/vision/active_mode", "intake");
        System.out.println("Carema Mode: " + mode);
        var idx = java.util.Arrays.asList(modes).indexOf(mode);
        if (idx<0) idx =0;
        idx = (idx + 1 ) % 3;
        SmartDashboard.putString("/vision/active_mode", modes[idx]);
        System.out.println("Selected Carema Mode: " + modes[idx]);
        responseTimer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
