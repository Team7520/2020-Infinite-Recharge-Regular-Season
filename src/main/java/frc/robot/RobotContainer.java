/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

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
import edu.wpi.cscore.UsbCamera;
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
  public TalonSRX liftMotor1;
  private TalonSRX liftMotor2;
  public TalonFX shooterMotor1;
  private TalonFX shooterMotor2;
  public TalonFX falcon500;
  private WPI_TalonSRX leftDrive1;
  private WPI_TalonSRX leftDrive2;
  private WPI_TalonSRX rightDrive1;
  private WPI_TalonSRX rightDrive2;

  private TalonSRX bufferMotor;
  private TalonSRX intakeMotor;

  private DoubleSolenoid intakeDrawerLeft;
  private DoubleSolenoid intakeDrawerRight;

  private UsbCamera camera1;
  private UsbCamera camera2;
  private UsbCamera camera3;

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem;
  //private final IntakeSub m_IntakeSub;
  private final ShooterSub m_ShooterSub;
  private final BufferSub m_BufferSub;
  private final LiftSub m_LiftSub;
  //private final Falcon500Sub m_Falcon500Sub;
  private final DriveTrain m_DriveTrain;
  private final CameraSub m_CameraSub;
  private final NavX m_NavX;
  public final PDP m_PDP;

  private final Command m_ShootCmd;
  //private final Command m_IntakeStartCmd;
  
  private final Command m_Buffer;
  private final ExampleCommand m_autoCommand;
  Command m_autonomousCommand;
  SendableChooser<Command> m_autoChooser = new SendableChooser<>();


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
    //falcon500 = new TalonFX(1);
    
    shooterMotor1 = new TalonFX(9);
    shooterMotor2 = new TalonFX(10);

    liftMotor1 = new TalonSRX(5);
    liftMotor2 = new TalonSRX(6);
    bufferMotor = new TalonSRX(7);
    intakeMotor = new TalonSRX(8);

    leftDrive1 = new WPI_TalonSRX(3);
    leftDrive2 = new WPI_TalonSRX(4);
    rightDrive1 = new WPI_TalonSRX(1);
    rightDrive2 = new WPI_TalonSRX(2);
    //intakeDrawerLeft = new DoubleSolenoid(0, 1);
    //intakeDrawerRight = new DoubleSolenoid(2, 3);

    m_exampleSubsystem = new ExampleSubsystem();
    m_NavX = new NavX(ahrs);
    //m_IntakeSub = new IntakeSub(intakeMotor, intakeDrawerLeft, intakeDrawerRight);
    m_ShooterSub = new ShooterSub(shooterMotor1, shooterMotor2);
    m_LiftSub = new LiftSub(liftMotor1, liftMotor2);

    m_BufferSub = new BufferSub(bufferMotor);
    //m_Falcon500Sub = new Falcon500Sub(falcon500);
     
    m_DriveTrain = new DriveTrain(leftDrive1, leftDrive2, rightDrive1, rightDrive2, m_NavX);
    m_CameraSub = new CameraSub(camera1, camera2, camera3);

    //m_IntakeStartCmd = new IntakeStartCommand(m_IntakeSub);
    m_ShootCmd = new Shoot(m_ShooterSub, XboxControl);
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
    button1.whileHeld(m_ShootCmd); //button1 and button2 move the shooter in opposite directions
    button2.whileHeld(m_ShootCmd);

    button5.whileHeld(m_Buffer); //button5 and button6 move the buffer (feeder) in opposite directions
    button6.whileHeld(m_Buffer);
  }

  public void configDashboard() {
    // To do: need a flag, for example, 'Test Mode', only when it is turned on, show various 
    // measures. Showing these measures on SmartDashBoard will cause loop time of 0.02s overrun

    double magVel_UnitsPer100ms = shooterMotor1.getSelectedSensorVelocity(0);
    //double falcon500Vel_UnitsPer100ms = falcon500.getSelectedSensorVelocity(0);
		/**
		 * Convert to RPM
		 * https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
		 * MagRPM = magVel [units/kT] * 600 [kTs/minute] / 4096(units/rev), where kT = 100ms
		 */
		double magVelRPM = magVel_UnitsPer100ms * 600 / 4096;
    //double shooterMotor1_RPM = falcon500Vel_UnitsPer100ms * 600 / 4096;
    double ratio = 2.2222;
    //double flyWhell_RPM = shooterMotor1_RPM * ratio; 
    double flyWhellRadius = 3; // inches
    //double ballOutSpeed = flyWhell_RPM * flyWhellRadius * 3.14159 / 12; 

    //SmartDashboard.putNumber("DriveTrain left QuadPosition", leftDrive1.getSensorCollection().getQuadraturePosition());
    //SmartDashboard.putNumber("DriveTrain Right QuadPosition", rightDrive1.getSensorCollection().getQuadraturePosition());
    SmartDashboard.putNumber("Encoder RPM", magVelRPM);
    SmartDashboard.putNumber("shooterMotor1 (Falcon) Abs Position", shooterMotor1.getSensorCollection().getIntegratedSensorAbsolutePosition());
//    SmartDashboard.putNumber("shooterMotor1 Position", falcon500.getSensorCollection().getIntegratedSensorPosition());
    // SmartDashboard.putNumber("shooterMotor1 RPM", shooterMotor1_RPM);
    // SmartDashboard.putNumber("Fly Wheel RPM", flyWhell_RPM);
    // SmartDashboard.putNumber("Ball Out Speed (feet)", ballOutSpeed);

    //SmartDashboard.putNumber("Encoder via DIO", encoder.getDistance();//getQuadraturePosition());
    SmartDashboard.putNumber("PDP Bus voltage", m_PDP.getVoltage());
    SmartDashboard.putNumber("PDP Temperature", m_PDP.getTemperature());
    SmartDashboard.putNumber("PDP Total Current", m_PDP.getTotalCurrent());
    SmartDashboard.putNumber("PDP Total Energy", m_PDP.getTotalEnergy());
    SmartDashboard.putNumber("PDP, Channel 0 (shooter 1), current", m_PDP.getCurrent(1));

    //SmartDashboard.putNumber("PDP Bus voltage", shooterMotor1.getsens .m_PDP.getVoltage());

  }

  public void configDashboard_new() {
/*    
    SmartDashboard.putString("Auton Chooser", " select before each game");
    //m_oi = new OI();
    m_autoChooser.addOption("Hatch In (up)", hatchIn.getInstance());
    m_autoChooser.addOption("Hatch Out (down)", hatchOut.getInstance());
    m_autoChooser.addOption("Hatch In and Out", CommandGroupTest.getInstance()); //testing commandgroups
    m_autoChooser.addOption("Collect Bar Up", new CollectBarRaising());
    m_autoChooser.addOption("Collect Bar Down", new CollectBarLowering());
    m_autoChooser.addOption("Cargo Intake", new CargoIntake());
    m_autoChooser.addOption("Cargo Shoot", new CargoShoot());
    m_autoChooser.addOption("Arm Up To Limit", new ArmToUpLimit());
    m_autoChooser.addOption("Auton Side Cargo Panel", new AutonSideCargoPanel());
    m_autoChooser.setDefaultOption("Cross Hab Line (Default Auto)", AutonCrossHabLine.getInstance());
*/
  }

  public void checkJoystick(){
    double driverLeftX = XboxControl.getRawAxis(0);
    double driverLeftY = XboxControl.getRawAxis(1);
    m_DriveTrain.move(driverLeftY, driverLeftX);

    double driverRightY = XboxControl.getRawAxis(5);
    if(Math.abs(driverRightY) > 0){
      m_ShootCmd.schedule();
    }
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
