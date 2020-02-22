/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
 
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // To do: need a flag, for example, 'Test Mode', only when it is turned on, show various 
    // measures. Showing these measures on SmartDashBoard will cause loop time of 0.02s overrun

    double magVel_UnitsPer100ms = m_robotContainer.shooterMotor1.getSelectedSensorVelocity(0);
    double falcon500Vel_UnitsPer100ms = m_robotContainer.Falcon500.getSelectedSensorVelocity(0);
		/**
		 * Convert to RPM
		 * https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
		 * MagRPM = magVel [units/kT] * 600 [kTs/minute] / 4096(units/rev), where kT = 100ms
		 */
		double magVelRPM = magVel_UnitsPer100ms * 600 / 4096;
		double falcon500Vel_RPM = falcon500Vel_UnitsPer100ms * 600 / 4096;

    SmartDashboard.putNumber("Encoder QuadPosition", m_robotContainer.shooterMotor1.getSensorCollection().getQuadraturePosition());
    SmartDashboard.putNumber("Encoder RPM", magVelRPM);
    SmartDashboard.putNumber("Falcon500 Abs Position", m_robotContainer.Falcon500.getSensorCollection().getIntegratedSensorAbsolutePosition());
    SmartDashboard.putNumber("Falcon500 Position", m_robotContainer.Falcon500.getSensorCollection().getIntegratedSensorPosition());
    SmartDashboard.putNumber("Falcon500 RPM", falcon500Vel_RPM);
    //SmartDashboard.putNumber("Encoder via DIO", m_robotContainer.encoder.getDistance();//getQuadraturePosition());
    SmartDashboard.putNumber("PDP Bus voltage", m_robotContainer.m_PDP.getVoltage());
    SmartDashboard.putNumber("PDP Temperature", m_robotContainer.m_PDP.getTemperature());
    SmartDashboard.putNumber("PDP Total Current", m_robotContainer.m_PDP.getTotalCurrent());
    SmartDashboard.putNumber("PDP Total Energy", m_robotContainer.m_PDP.getTotalEnergy());
    SmartDashboard.putNumber("PDP, Channel 0 (shooter 1), current", m_robotContainer.m_PDP.getCurrent(1));

    //SmartDashboard.putNumber("PDP Bus voltage", m_robotContainer.shooterMotor1.getsens .m_PDP.getVoltage());

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
   
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
