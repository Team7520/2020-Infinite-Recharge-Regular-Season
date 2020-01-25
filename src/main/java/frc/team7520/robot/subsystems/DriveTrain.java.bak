/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team7520.robot.subsystems;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;


import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team7520.robot.Robot;
/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  private TalonSRX motorLeftMaster;
  private TalonSRX motorRightMaster;
  private DifferentialDrive drive; 

  private double startYaw = 0;
  private double currentYaw = 0 ;
  private boolean isMovingStraight = false;
    // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public void setLeftMasterMotor(TalonSRX leftMasterMotor) {
    motorLeftMaster = leftMasterMotor;
  }

  public void setRightMasterMotor(TalonSRX rightMasterMotor) {
    motorRightMaster = rightMasterMotor;
  }

  public void setDriveTrain(WPI_TalonSRX leftMasterMotor, WPI_TalonSRX rightMasterMotor) {
    drive = new DifferentialDrive(leftMasterMotor, rightMasterMotor);
  }

  public void setStartYaw(double yaw) {
    startYaw = yaw;
  }

  public void move(double driveSpeed, double zRotation) {
    isMovingStraight = false;
    // motorLeftMaster.set(ControlMode.PercentOutput, leftSpeed);
    // motorRightMaster.set(ControlMode.PercentOutput, rightSpeed);
    drive.arcadeDrive(driveSpeed, zRotation);
  }

  public void moveStraight(double driveSpeed) {
    // if(!isMovingStraight)
    // {  
    //   startYaw = Robot.navX.getZAngle();
    //   isMovingStraight = true;
    // }
    // currentYaw = Robot.navX.getZAngle();
    // double errorAllowance = 1.0; // degree
    // // get current yaw
    // if(Math.abs(currentYaw - startYaw) > errorAllowance) 
    // {
    //   double speedAdjustment = leftSpeed * 0.1 * (currentYaw - startYaw) / Math.abs(currentYaw - startYaw);
    //   motorLeftMaster.set(ControlMode.PercentOutput, leftSpeed * (1 - speedAdjustment));
    //   motorRightMaster.set(ControlMode.PercentOutput, leftSpeed);
    // }
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
