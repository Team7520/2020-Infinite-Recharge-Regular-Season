/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.CameraReflectLight;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Timer;

/**
 * A command to turn the camera reflective light on and off using the CameraReflectLight subsystem.
 */
public class CameraLightCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CameraReflectLight light;
  private Timer responseTimer;
  private JoystickButton toggle;
  private boolean lightStatus;
  private final double delayTime = 0.5;

  /**
   * Creates a new CameraLightCommand.
   *
   * @param light The CameraReflectLight used by this command.
   * @param toggle The button to toggle the light on and off.
   */
  public CameraLightCommand(CameraReflectLight light, JoystickButton toggle) {
    this.light = light;
    addRequirements(light);

    this.toggle = toggle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    responseTimer = new Timer();
    responseTimer.start();
    lightStatus = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(toggle.get() && responseTimer.hasElapsed(delayTime)){
        lightStatus = !lightStatus;
        responseTimer.reset();
    }

    if(lightStatus){
        light.lightOn();
    } else {
        light.lightOff();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    light.lightOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
