/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team7520.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.team7520.robot.Robot;
import frc.team7520.robot.sensors.NavX;
import frc.team7520.robot.utilities.SpeedPlan;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {

  public WPI_TalonSRX leftMasterMotor;
  private WPI_VictorSPX leftFollowerMotor;
  public WPI_TalonSRX rightMasterMotor;
  private WPI_VictorSPX rightFollowerMotor; 

  private static DifferentialDrive drive; 


  private double startYaw = 0;
  private double currentYaw = 0 ;
  private boolean isMovingStraight = false;
  
  public SpeedPlan turnRadiusSpeedPlan = new SpeedPlan();
  public SpeedPlan moveStraightSpeedPlan = new SpeedPlan();

  public boolean isInAuto = false; 

  // Put methods for controlling this subsystem
  // here. Call these from Commands.


	public void setSpeed(double speed)
	{
		drive.tankDrive(speed, speed);
	}

	public void setSpeed(double leftSpeed, double rightSpeed)
	{
		drive.tankDrive(leftSpeed, rightSpeed);
	}

	public void stop()
	{
		drive.stopMotor();
	}


  /** 
   * Sets the robot's left master and follower motors.
  */

  public void setLeftMotorGroup(WPI_TalonSRX leftMasterMotor, WPI_VictorSPX leftFollowerMotor) {
    this.leftMasterMotor = leftMasterMotor;
    this.leftFollowerMotor = leftFollowerMotor;
    this.leftFollowerMotor.set(ControlMode.Follower, this.leftMasterMotor.getDeviceID());
  }
  /** 
   * Sets the robot's right master and follower motors.
  */

  public void setRightMotorGroup(WPI_TalonSRX rightMasterMotor, WPI_VictorSPX rightFollowerMotor) {
    this.rightMasterMotor = rightMasterMotor;
    this.rightFollowerMotor = rightFollowerMotor;
    this.rightFollowerMotor.set(ControlMode.Follower, this.rightMasterMotor.getDeviceID());
  }

  /** 
   * Creates a differential drive object with the two master motors as parameters.
  */

  public DifferentialDrive getDriveTrain() {
	  if(drive == null) {
		drive = new DifferentialDrive(leftMasterMotor, rightMasterMotor);
	  }
	  return drive;		
  }

  public void setStartYaw(double yaw) {
    startYaw = yaw;
  }

  public void move(double driveSpeed, double zRotation) {
    isMovingStraight = false;
    drive.arcadeDrive(driveSpeed, zRotation);
  }

  public void moveStraight(double driveSpeed) {
    if(!isMovingStraight)
    {  
      startYaw = Robot.navX.getZAngle();
      isMovingStraight = true;
    }
    currentYaw = Robot.navX.getZAngle();
    double errorAllowance = 1.0; // degree
    // get current yaw
    if(Math.abs(currentYaw - startYaw) > errorAllowance) 
    {
      // double speedAdjustment = leftSpeed * 0.1 * (currentYaw - startYaw) / Math.abs(currentYaw - startYaw);
      // motorLeftMaster.set(ControlMode.PercentOutput, leftSpeed * (1 - speedAdjustment));
      // motorRightMaster.set(ControlMode.PercentOutput, leftSpeed);
	  drive.arcadeDrive(driveSpeed, (startYaw - currentYaw)); //the rotation is equal to -(currentYaw - startYaw)
    } else {
		drive.arcadeDrive(driveSpeed, 0);	
	}
  }

  	public void configSpeedPlan() {
		turnRadiusSpeedPlan.init(0.2, 0.3, 0.15, 40, 10);
		moveStraightSpeedPlan.init(0.27, 0.9, 0.15, 30, 17);
	}
  
	public void turnRightRadius(double speed, double angle, double radius, double width)
	{
		turnRadius( speed,  angle,  radius,  width);
	}
	
	public void turnLeftRadius(double speed, double angle, double radius, double width)
	{
		turnRadius( speed,  -angle,  radius,  width);
	}

	public void turnRadius(double speed, double angle, double radius, double width)
	{
//		rightJoystick = new HBJoystick(0);

//11		leftJoystick = new HBJoystick(1);

//		isSwitched = false;
		
		NavX navX = Robot.navX;
		navX.resetLastRawYaw();
		double InnerSpeed = speed*((radius-width/2)/(radius+width/2));
		double currentYaw = navX.getTrueYaw();
		double endAngle = currentYaw + angle;
		System.out.println("+------------------------------+");
		System.out.println("turnRadius: speed =" + speed + "; angle=" + angle);
		System.out.println("Start Angle: " + currentYaw);
		System.out.println("End Angle: " + endAngle);
		System.out.println("InnerSpeed: " + InnerSpeed);
//		if(angle > 0) // up right
//			ActuatorConfig.getInstance().getDrivetrain().setSpeed(speed, InnerSpeed);
//		else // up left
//			ActuatorConfig.getInstance().getDrivetrain().setSpeed(InnerSpeed, speed);
		
		// down right
		//  ActuatorConfig.getInstance().getDrivetrain().setSpeed(-InnerSpeed, -speed);
		// down left
		//	ActuatorConfig.getInstance().getDrivetrain().setSpeed(-speed, -InnerSpeed);

		//???		ActuatorConfig.getInstance().getDrivetrain().setSpeed(speed/2, -speed/2);

		double lastYaw = currentYaw;
		double startYaw = currentYaw;
		double currentSpeed = 0;
		double turnedAngle = 0;
		
		if(Math.abs(angle) <= 1)
			{
				System.out.println("Hard to rotate such a small angle: " + angle);
				return;
			}
			while ((angle > 0 && currentYaw < endAngle)|| 
			(angle < 0 && currentYaw > endAngle))
			{
				currentYaw = navX.getTrueYaw();
				turnedAngle = Math.abs(currentYaw - startYaw);
				currentSpeed = turnRadiusSpeedPlan.getSpeed(Math.abs(angle), speed, turnedAngle);
				InnerSpeed = currentSpeed*((radius-width/2)/(radius+width/2));
				if(angle > 0) // up right
//					ActuatorConfig.getInstance().getDrivetrain().setSpeed(currentSpeed, InnerSpeed);
					setSpeed(currentSpeed, InnerSpeed);
				else 
//					ActuatorConfig.getInstance().getDrivetrain().setSpeed(-currentSpeed, -InnerSpeed);
					setSpeed(-currentSpeed, -InnerSpeed);

				//if (Math.abs(currentYaw - lastYaw) > 10) //NavX has a bug: Yaw values suddenly change signs
				//	currentYaw = -currentYaw;
				//if(lastYaw < currentYaw - 0.5 || lastYaw > currentYaw + 0.5)
				//	   System.out.println("current Angle: " + currentYaw);
				// if (RobotStatus.isAuto() && (AutonStatus.getInstance().getStatus() == Status.CANCELED))
				// {
				// 	break;
				// }
				// if (RobotStatus.isTeleop() && turnRadiusCancel)
				// {
				// 	break;
				// }
				lastYaw = currentYaw;

			}
//		}
		System.out.println("After turning: " + currentYaw);
//		ActuatorConfig.getInstance().getDrivetrain().stop();
		stop();

		currentYaw = navX.getTrueYaw();
		System.out.println("After stopped angle: " + currentYaw);
		System.out.println("--------------------");

	}
	
	public void moveGyro(double distance, double angle, double speed,  boolean isReversed)
	{
		NavX navx = Robot.navX;

		boolean isRightComplete = false;
		boolean isLeftComplete = false;
		double distanceRight;
		double distanceLeft;
		double rightEncoderValue = rightMasterMotor.getSensorCollection()
				.getQuadraturePosition() / (2048.0);// 8192 is the ppr of the
													// encoder x4
		double leftEncoderValue = leftMasterMotor.getSensorCollection()
				.getQuadraturePosition() / (-2048.0);
		double rightEncoderStartValue = rightEncoderValue;
		double leftEncoderStartValue = leftEncoderValue;
		
//		double deaccIdealDistance
//		double 
		
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

		double currentYaw;
		double startYaw = navx.getRawYaw();
		double targetYaw = startYaw + angle;
		 SmartDashboard.putNumber("Start Yaw: ", startYaw);
		setSpeed(speed);
		System.out.println("Start Yaw: " + startYaw);
		System.out.println("Target Yaw: " + targetYaw);
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
//		resetDriveTrainEncoderSamplings();
//		double estimateTime = estimateTimeUsed(distance, 0, speed);

		//while ((!isRightComplete || !isLeftComplete) && RobotStatus.isAuto() && RobotStatus.isRunning())
//		while (!isRightComplete && !isLeftComplete && RobotStatus.isAuto() && RobotStatus.isRunning())
		while (!isRightComplete)
		{

			rightEncoderValue = rightMasterMotor.getSensorCollection()
					.getQuadraturePosition() / (2048.0);
			leftEncoderValue = leftMasterMotor.getSensorCollection()
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
			currentSpeed = this.moveStraightSpeedPlan.getSpeed(Math.abs(distance), speed, movedDistance);
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
					setSpeed(currentSpeed, (currentSpeed + Math.abs(currentSpeed)/10));
				}
			 //???else if (currentYaw < (startYaw + 0.5))
				else if (currentYaw < (targetYaw - 1))
				{
					// Veering right, so slow down left
					// System.out.println("Veering right");
//					ActuatorConfig.getInstance().getDrivetrain().setSpeed((speed+ .15), speed);
					//ActuatorConfig.getInstance().getDrivetrain().setSpeed((currentSpeed + Math.abs(currentYaw - targetYaw)/30), currentSpeed);
					setSpeed((currentSpeed + Math.abs(currentSpeed)/10), currentSpeed);
				}
				else 
					setSpeed(currentSpeed);
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
					setSpeed(currentSpeed, (currentSpeed + Math.abs(currentSpeed)/10));

				//???} else if (currentYaw < (startYaw + 1))
			    } else if (currentYaw < (targetYaw - 1))
				{
					// Veering right, so slow down left
					// System.out.println("Veering right");

//					ActuatorConfig.getInstance().getDrivetrain().setSpeed((speed + .12), speed);// 0.05,
																								// +.16
					//ActuatorConfig.getInstance().getDrivetrain().setSpeed((currentSpeed + Math.abs(currentYaw - targetYaw)/30), currentSpeed);
					setSpeed((currentSpeed + Math.abs(currentSpeed)/10), currentSpeed);
				}
			    else
			    	setSpeed(currentSpeed, currentSpeed);
			}
		}
		rightEncoderValue = rightMasterMotor.getSensorCollection()
				.getQuadraturePosition() / (2048.0);
		leftEncoderValue = leftMasterMotor.getSensorCollection()
				.getQuadraturePosition() / (-2048.0);
		currentYaw = navx.getRawYaw();
		
		System.out.println("End Yaw: " + currentYaw);
		System.out.println("Left Encoder End Value: " + leftEncoderValue);
		System.out.println("Right Encoder End Value: " + rightEncoderValue);
		
		stop();
		
		rightEncoderValue = rightMasterMotor.getSensorCollection()
				.getQuadraturePosition() / (2048.0);
		leftEncoderValue = leftMasterMotor.getSensorCollection()
				.getQuadraturePosition() / (-2048.0);
		
	
		System.out.println("Left Encoder Moved Value: " + (leftEncoderValue - leftEncoderStartValue));
		System.out.println("Right Encoder Moved Value: " + (rightEncoderValue- rightEncoderStartValue));
//		System.out.println("Estimated Time: " + String.format("%.3f", estimateTime));
//		System.out.println("Time used: " + String.format("%.3f", (double)((currentTimeMillis - startTimeMillis)/1000)));

//		SensorConfig.getInstance().getTimer().waitTimeInMillis(200); //???
	}
	public void goForwardGyro(double distance, double angle, double speed )
	{
		System.out.println("Forward: d=" + distance + "; angle=" + angle + "speed=" + speed);
		moveGyro(distance, angle, speed, false);
		System.out.println("--------------------");
	
	}
	
	public void goBackwardsGyro(double distance, double angle, double speed)
	{ 
		System.out.println("BackwardsGyro: d=" + distance + "; angle=" + angle + "speed=" + speed);
		moveGyro(distance, angle, speed, true);
		System.out.println("--------------------");
	}

	@Override
  	public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  	}
}