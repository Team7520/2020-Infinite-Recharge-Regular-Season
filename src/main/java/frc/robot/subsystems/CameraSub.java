/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

public class CameraSub extends SubsystemBase {
  /**
   * Creates a new CameraSub.
   */
  public CameraSub(UsbCamera... cameras) { //unlimited camera parameters; we usually only have around 3 usb cameras, though.
    for(int i = 0; i < cameras.length; i++){
      cameras[i] = CameraServer.getInstance().startAutomaticCapture("Camera " + i, i);
      cameras[i].setResolution(160, 120);
      //cameras[i].setFPS(30);
      cameras[i].setFPS(10);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
