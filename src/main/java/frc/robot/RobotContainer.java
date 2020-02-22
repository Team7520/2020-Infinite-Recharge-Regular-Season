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

import edu.wpi.first.wpilibj.Encoder;
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
  private JoystickButton button2;

  private JoystickButton button5;
  private JoystickButton button6;

  private AHRS ahrs;
  public  Encoder encoder;

  //private TalonSRX testMotor1;
  public TalonSRX shooterMotor1;
  private TalonSRX shooterMotor2;
  public TalonFX Falcon500;
  // private WPI_TalonSRX leftDrive1;
  // private WPI_TalonSRX leftDrive2;
  // private WPI_TalonSRX rightDrive1;
  // private WPI_TalonSRX rightDrive2;

  private VictorSPX bufferMotor;

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem;
  private final IntakeSub m_IntakeSub;
  private final BufferSub m_BufferSub;
  //private final DriveTrain m_DriveTrain;
  private final NavX m_NavX;
  public final PDP m_PDP;

  private final Command m_Shoot;
  private final Command m_Buffer;
  private final ExampleCommand m_autoCommand;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    XboxControl = new Joystick(1);
    // public Button button1 = new Button() {
    //   BooleanSupplier sup = () -> XboxControl.getRawButton(0);
    // };
    button1 = new JoystickButton(XboxControl, 1);
    button2 = new JoystickButton(XboxControl, 2);

    button5 = new JoystickButton(XboxControl, 5);
    button6 = new JoystickButton(XboxControl, 6);

    m_PDP = new PDP(); 
    ahrs = new AHRS(SPI.Port.kMXP);

    encoder = new Encoder(0, 1);

    //testMotor1 = new TalonSRX(1);
    Falcon500 = new TalonFX(1);

    shooterMotor1 = new TalonSRX(5);
    shooterMotor2 = new TalonSRX(3);
    bufferMotor = new VictorSPX(2);
    // leftDrive1 = new WPI_TalonSRX(3);
    // leftDrive2 = new WPI_TalonSRX(4);
    // rightDrive1 = new WPI_TalonSRX(5);
    // rightDrive2 = new WPI_TalonSRX(6);


    m_exampleSubsystem = new ExampleSubsystem();
    m_NavX = new NavX(ahrs);
    m_IntakeSub = new IntakeSub(shooterMotor1, shooterMotor2);
    //m_BufferSub = new BufferSub(bufferMotor);
    m_BufferSub = new BufferSub(Falcon500);
     
    //m_DriveTrain = new DriveTrain(leftDrive1, leftDrive2, rightDrive1, rightDrive2, m_NavX);

    m_Shoot = new Shoot(m_IntakeSub);
    m_Buffer = new Buffer(m_BufferSub);
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
    button5.whileHeld(m_Buffer);
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
