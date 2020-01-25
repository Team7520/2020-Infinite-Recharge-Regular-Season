/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team7520.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team7520.robot.Robot;
import frc.team7520.robot.sensors.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MoveGyro extends Command {

  NavX navx = Robot.navX;

  double timeout = 10.0;

  double distance; 
  double angle;
  double speed;  
  boolean isReversed;

		boolean isRightComplete = false;
		boolean isLeftComplete = false;
		double distanceRight;
		double distanceLeft;
		double rightEncoderValue = Robot.m_driveTrain.rightMasterMotor.getSensorCollection()
				.getQuadraturePosition() / (2048.0);// 8192 is the ppr of the
													// encoder x4
		double leftEncoderValue = Robot.m_driveTrain.leftMasterMotor.getSensorCollection()
				.getQuadraturePosition() / (-2048.0);
		double rightEncoderStartValue = rightEncoderValue;
		double leftEncoderStartValue = leftEncoderValue;
		
//		double deaccIdealDistance
//		double 

		double currentYaw;
		double startYaw = navx.getRawYaw();
		double targetYaw = startYaw + angle;
		// SmartDashboard.putNumber("Right Distance To ", distanceRight);
		// SmartDashboard.putNumber("Left Distance To ", distanceLeft);
		// SmartDashboard.putNumber("Start Left Enoder Value ",
		// leftEncoderValue);
		// SmartDashboard.putNumber("Start Right Encoder Value",
		// rightEncoderValue);

		//??? following condition let left and right stop to move at same time,
		//??? any of them complete, then the other one also stops moving.
		//??? we need both completes, i.e., in front of switch, let robot touch switch by whole front frame.
		double movedDistance = 0;
    double currentSpeed = 0;
    
    //private static MoveGyro instance;

  public MoveGyro(double distance, double angle, double speed,  boolean isReversed) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_driveTrain);
    setTimeout(timeout);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Left Encoder Start Value: " + leftEncoderValue);
    System.out.println("Right Encoder Start Value: " + rightEncoderValue);
    if (isReversed)
		{
			speed = speed * -1;
			distanceRight = rightEncoderValue - distance;
			distanceLeft = leftEncoderValue - distance;
		} else
		{
			distanceRight = rightEncoderValue + distance;
			distanceLeft = leftEncoderValue + distance;
    }
    
    SmartDashboard.putNumber("Start Yaw: ", startYaw);
		Robot.m_driveTrain.setSpeed(speed);
		System.out.println("Start Yaw: " + startYaw);
		System.out.println("Target Yaw: " + targetYaw);
  }
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    rightEncoderValue = Robot.m_driveTrain.rightMasterMotor.getSensorCollection()
					.getQuadraturePosition() / (2048.0);
			leftEncoderValue = Robot.m_driveTrain.leftMasterMotor.getSensorCollection()
					.getQuadraturePosition() / (-2048.0);

//			SmartDashboard.putNumber("Left Encoder Value ", leftEncoderValue);
//			SmartDashboard.putNumber("Right Encoder Value", rightEncoderValue);

			movedDistance = Math.abs(rightEncoderValue - rightEncoderStartValue);
			if(Math.abs(movedDistance) > Math.abs(distance) / 3)
			{
//				robotVelocityXSamplings.addSample(navx.getVelocityX());
//				leftEncoderSamplings.addSample(leftEncoderValue);
//				rightEncoderSamplings.addSample(rightEncoderValue);
				// if(robotBlockedByObstacle(estimateTime))
				// {
				// 	System.out.println("Robot is blocked by obstacle(s), quit moving.");
				// 	break;
				// }
			}

			// used wrong speed plan during MARC event; low speed caused the robot to be stuck during auton
//			currentSpeed = turnRadiusSpeedPlan.getSpeed(Math.abs(distance), speed, movedDistance);
			currentSpeed = Robot.m_driveTrain.moveStraightSpeedPlan.getSpeed(Math.abs(distance), speed, movedDistance);
			if (isReversed)
			{
				currentSpeed = currentSpeed * -1;
			}
			// This Kill Switch will only work once Teleop begins and Joysticks
			// start working
			// This is a safety in case loop cannot complete for some reason
			// while running Auton.
			// To test: Run Auton and hit disable before lop completes. Then
			// start teleop and press kill switch buttons
			/*
			 * if(rightJoystick.getRawButton(1) || leftJoystick.getRawButton(1))
			 * { System.out.println("Kill Switch"); break; }
			 */

			if (isReversed)
			{
				
				if (rightEncoderValue <= distanceRight)
				{
					isRightComplete = true;
					// ActuatorConfig.getInstance().getDrivetrain().getLeftMotor().stop();
					
				}
				if (leftEncoderValue <= distanceLeft)
				{
					isLeftComplete = true;
					// ActuatorConfig.getInstance().getDrivetrain().getLeftMotor().stop();
				}
				currentYaw = navx.getRawYaw();
//				SmartDashboard.putNumber("Current Yaw ", currentYaw);
				if (currentYaw > (targetYaw + 1))
				{
					// Veering left, so slow down right
					// System.out.println("Veering left");
//					ActuatorConfig.getInstance().getDrivetrain().setSpeed(speed, (speed+ .15));
					//ActuatorConfig.getInstance().getDrivetrain().setSpeed(speed, (speed + Math.abs(currentYaw - targetYaw)/30));
					//ActuatorConfig.getInstance().getDrivetrain().setSpeed(currentSpeed, (currentSpeed + Math.abs(currentYaw - targetYaw)/30));
					Robot.m_driveTrain.setSpeed(currentSpeed, (currentSpeed + Math.abs(currentSpeed)/10));
				}
			 //???else if (currentYaw < (startYaw + 0.5))
				else if (currentYaw < (targetYaw - 1))
				{
					// Veering right, so slow down left
					// System.out.println("Veering right");
//					ActuatorConfig.getInstance().getDrivetrain().setSpeed((speed+ .15), speed);
					//ActuatorConfig.getInstance().getDrivetrain().setSpeed((currentSpeed + Math.abs(currentYaw - targetYaw)/30), currentSpeed);
					Robot.m_driveTrain.setSpeed((currentSpeed + Math.abs(currentSpeed)/10), currentSpeed);
				}
				else 
        Robot.m_driveTrain.setSpeed(currentSpeed);
			} else
			{
				if (rightEncoderValue >= distanceRight)
				{
					isRightComplete = true;
					// ActuatorConfig.getInstance().getDrivetrain().getLeftMotor().stop();
					//System.out.println("Left Finished First");
				}
				if (leftEncoderValue >= distanceLeft)
				{
					isLeftComplete = true;
					
				}
				currentYaw = navx.getRawYaw();
//				SmartDashboard.putNumber("Current Yaw ", currentYaw);
				if (currentYaw > (targetYaw + 1))
				{
					// Veering left, so slow down right
					// System.out.println("Veering left");

//					ActuatorConfig.getInstance().getDrivetrain().setSpeed(speed, (speed + .12));// 0.05,
																								// +.16
					//ActuatorConfig.getInstance().getDrivetrain().setSpeed(currentSpeed, (currentSpeed + Math.abs(currentYaw - targetYaw)/30));
					Robot.m_driveTrain.setSpeed(currentSpeed, (currentSpeed + Math.abs(currentSpeed)/10));

				//???} else if (currentYaw < (startYaw + 1))
			    } else if (currentYaw < (targetYaw - 1))
				{
					// Veering right, so slow down left
					// System.out.println("Veering right");

//					ActuatorConfig.getInstance().getDrivetrain().setSpeed((speed + .12), speed);// 0.05,
																								// +.16
					//ActuatorConfig.getInstance().getDrivetrain().setSpeed((currentSpeed + Math.abs(currentYaw - targetYaw)/30), currentSpeed);
					Robot.m_driveTrain.setSpeed((currentSpeed + Math.abs(currentSpeed)/10), currentSpeed);
				}
			    else
          Robot.m_driveTrain.setSpeed(currentSpeed, currentSpeed);
			}
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_driveTrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
