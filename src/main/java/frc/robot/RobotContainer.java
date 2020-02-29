/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;


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

  private JoystickButton button5; //using these two for our buffer system
  private JoystickButton button6;

  private AHRS ahrs;
  public  Encoder encoder;

  //private TalonSRX testMotor1;
  public TalonSRX shooterMotor1;
  private TalonSRX shooterMotor2;
  public TalonFX falcon500;
  // private WPI_TalonSRX leftDrive1;
  // private WPI_TalonSRX leftDrive2;
  // private WPI_TalonSRX rightDrive1;
  // private WPI_TalonSRX rightDrive2;

  private VictorSPX bufferMotor;
  private VictorSPX intakeMotor;

  private DoubleSolenoid intakeDrawerLeft;
  private DoubleSolenoid intakeDrawerRight;


  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem;
  private final IntakeSub m_IntakeSub;
  private final ShooterSub m_ShooterSub;
  private final BufferSub m_BufferSub;
  //private final Falcon500Sub m_Falcon500Sub;
  //private final DriveTrain m_DriveTrain;
  private final NavX m_NavX;
  public final PDP m_PDP;

  private final Command m_ShootCmd;
  private final Command m_IntakeStartCmd;
  
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

    //encoder = new Encoder(0, 1);

    //testMotor1 = new TalonSRX(1);
    falcon500 = new TalonFX(1);

    shooterMotor1 = new TalonSRX(5);
    shooterMotor2 = new TalonSRX(3);
    bufferMotor = new VictorSPX(2);
    intakeMotor = new VictorSPX(4);

    // leftDrive1 = new WPI_TalonSRX(3);
    // leftDrive2 = new WPI_TalonSRX(4);
    // rightDrive1 = new WPI_TalonSRX(5);
    // rightDrive2 = new WPI_TalonSRX(6);
    intakeDrawerLeft = new DoubleSolenoid(0, 1);
    intakeDrawerRight = new DoubleSolenoid(2, 3);

    m_exampleSubsystem = new ExampleSubsystem();
    m_NavX = new NavX(ahrs);
    m_IntakeSub = new IntakeSub(intakeMotor, intakeDrawerLeft, intakeDrawerRight);
    m_ShooterSub = new ShooterSub(shooterMotor1, shooterMotor2);
    m_BufferSub = new BufferSub(falcon500);
    //m_Falcon500Sub = new Falcon500Sub(falcon500);
     
    //m_DriveTrain = new DriveTrain(leftDrive1, leftDrive2, rightDrive1, rightDrive2, m_NavX);
    //m_DriveTrain.setLeftMotorGroup(leftDrive1, leftDrive2);
    //m_DriveTrain.setRightMotorGroup(rightDrive1, rightDrive2);

    m_IntakeStartCmd = new IntakeStartCommand(m_IntakeSub);
    m_ShootCmd = new Shoot(m_ShooterSub);
    m_Buffer = new Buffer(m_BufferSub, button5, button6);
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
    button1.whileHeld(m_ShootCmd);
    button5.whileHeld(m_Buffer); //button5 and button6 move the buffer in opposite directions
    button6.whileHeld(m_Buffer);
  }

  public void configDashboard() {
    // To do: need a flag, for example, 'Test Mode', only when it is turned on, show various 
    // measures. Showing these measures on SmartDashBoard will cause loop time of 0.02s overrun

    double magVel_UnitsPer100ms = shooterMotor1.getSelectedSensorVelocity(0);
    double falcon500Vel_UnitsPer100ms = falcon500.getSelectedSensorVelocity(0);
		/**
		 * Convert to RPM
		 * https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
		 * MagRPM = magVel [units/kT] * 600 [kTs/minute] / 4096(units/rev), where kT = 100ms
		 */
		double magVelRPM = magVel_UnitsPer100ms * 600 / 4096;
		double falcon500Vel_RPM = falcon500Vel_UnitsPer100ms * 600 / 4096;

    SmartDashboard.putNumber("Encoder QuadPosition", shooterMotor1.getSensorCollection().getQuadraturePosition());
    SmartDashboard.putNumber("Encoder RPM", magVelRPM);
    SmartDashboard.putNumber("Falcon500 Abs Position", falcon500.getSensorCollection().getIntegratedSensorAbsolutePosition());
    SmartDashboard.putNumber("Falcon500 Position", falcon500.getSensorCollection().getIntegratedSensorPosition());
    SmartDashboard.putNumber("Falcon500 RPM", falcon500Vel_RPM);
    //SmartDashboard.putNumber("Encoder via DIO", encoder.getDistance();//getQuadraturePosition());
    SmartDashboard.putNumber("PDP Bus voltage", m_PDP.getVoltage());
    SmartDashboard.putNumber("PDP Temperature", m_PDP.getTemperature());
    SmartDashboard.putNumber("PDP Total Current", m_PDP.getTotalCurrent());
    SmartDashboard.putNumber("PDP Total Energy", m_PDP.getTotalEnergy());
    SmartDashboard.putNumber("PDP, Channel 0 (shooter 1), current", m_PDP.getCurrent(1));

    //SmartDashboard.putNumber("PDP Bus voltage", shooterMotor1.getsens .m_PDP.getVoltage());

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
