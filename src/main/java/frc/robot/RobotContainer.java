/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.BooleanSupplier;

import com.kauailabs.navx.frc.AHRS;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  private Joystick XboxControl;
  // public Button button1 = new Button();
  private JoystickButton button1;

  public AHRS ahrs;

  private TalonSRX testMotor1;
  private TalonSRX intakeMotor1;
  private TalonSRX intakeMotor2;
  private WPI_TalonSRX leftDrive1;
  private WPI_VictorSPX leftDrive2;
  private WPI_TalonSRX rightDrive1;
  private WPI_VictorSPX rightDrive2;

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem;
  private final IntakeSub m_IntakeSub;
  private final DriveTrain m_DriveTrain;
  private final NavX m_NavX;

  private final Command m_Shoot;
  private final ExampleCommand m_autoCommand;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    XboxControl = new Joystick(1);
    // public Button button1 = new Button() {
    //   BooleanSupplier sup = () -> XboxControl.getRawButton(0);
    // };
    button1 = new JoystickButton(XboxControl, 0);

    ahrs = new AHRS(SPI.Port.kMXP);

    testMotor1 = new TalonSRX(1);
    intakeMotor1 = new TalonSRX(0);
    intakeMotor2 = new TalonSRX(2);
    leftDrive1 = new WPI_TalonSRX(3);
    leftDrive2 = new WPI_VictorSPX(4);
    rightDrive1 = new WPI_TalonSRX(5);
    rightDrive2 = new WPI_VictorSPX(6);


    m_exampleSubsystem = new ExampleSubsystem();
    m_NavX = new NavX(ahrs);
    m_IntakeSub = new IntakeSub(intakeMotor1, intakeMotor2);
    m_DriveTrain = new DriveTrain(leftDrive1, leftDrive2, rightDrive1, rightDrive2, m_NavX);

    m_Shoot = new Shoot(m_IntakeSub);
    m_autoCommand = new ExampleCommand(m_exampleSubsystem);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //button1.whileHeld(new Shoot(m_IntakeSub));
    button1.whileHeld(m_Shoot);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
