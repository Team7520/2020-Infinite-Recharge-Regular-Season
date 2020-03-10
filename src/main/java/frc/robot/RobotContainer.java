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
  
  private Joystick DriverControl;
  private Joystick OperatorControl;
  // public Button driveButton1 = new Button();
  private JoystickButton driveButton1;
  private JoystickButton driveButton2;
  private JoystickButton driveButton3;
  private JoystickButton driveButton4;
  private JoystickButton driveButton5; //using these two for our buffer/feeder system
  private JoystickButton driveButton6;

  private JoystickButton opButton1;
  private JoystickButton opButton2;
  private JoystickButton opButton3;
  private JoystickButton opButton4;
  private JoystickButton opButton5;
  private JoystickButton opButton6;


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

  private TalonSRX feederMotor;
  private TalonSRX intakeMotor;

  private Solenoid intakeSingleSolenoid;
  //private DoubleSolenoid intakeDoubleSolenoid;
  //private DoubleSolenoid intakeDrawerRight;
  private DoubleSolenoid liftDoubleSolenoid;

  private Solenoid cameraReflectLightSolenoid;

  private UsbCamera camera1;
  private UsbCamera camera2;
  private UsbCamera camera3;

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem;
  private final IntakeSub m_IntakeSub;
  private final ShooterSub m_ShooterSub;
  private final FeederSub m_FeederSub;
  private final LiftSub m_LiftSub;
  private final CameraReflectLight m_CameraReflectLight;
  //private final Falcon500Sub m_Falcon500Sub;
  private final DriveTrain m_DriveTrain;
  private final CameraSub m_CameraSub;
  private final NavX m_NavX;
  public final PDP m_PDP;

  private final Command m_ShootCommand;
  private final Command m_IntakeTestCmd;
  private final Command m_LiftCmd;
  private final Command m_CameraLightCmd;
  private final Command m_FeedCommand;
  private final ExampleCommand m_autoCommand;
  Command m_autonomousCommand;
  SendableChooser<Command> m_autoChooser = new SendableChooser<>();


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    DriverControl = new Joystick(1);
    OperatorControl = new Joystick(2);
    // public Button driveButton1 = new Button() {
    //   BooleanSupplier sup = () -> DriverControl.getRawButton(0);
    // };
    driveButton1 = new JoystickButton(DriverControl, 1);
    driveButton2 = new JoystickButton(DriverControl, 2);
    driveButton3 = new JoystickButton(DriverControl, 3);
    driveButton4 = new JoystickButton(DriverControl, 4);
    driveButton5 = new JoystickButton(DriverControl, 5);
    driveButton6 = new JoystickButton(DriverControl, 6);

    opButton1 = new JoystickButton(OperatorControl, 1);
    opButton2 = new JoystickButton(OperatorControl, 2);
    opButton3 = new JoystickButton(OperatorControl, 3);
    opButton4 = new JoystickButton(OperatorControl, 4);
    opButton5 = new JoystickButton(OperatorControl, 5);
    opButton6 = new JoystickButton(OperatorControl, 6);
    

    m_PDP = new PDP(); 
    ahrs = new AHRS(SPI.Port.kMXP);

    //encoder = new Encoder(0, 1);

    //testMotor1 = new TalonSRX(1);
    //falcon500 = new TalonFX(1);
    
    shooterMotor1 = new TalonFX(9);
    shooterMotor2 = new TalonFX(10);

    liftMotor1 = new TalonSRX(3);
    liftMotor2 = new TalonSRX(4);
    feederMotor = new TalonSRX(2);
    intakeMotor = new TalonSRX(1); 

    leftDrive1 = new WPI_TalonSRX(7); // Left back (master)
    leftDrive2 = new WPI_TalonSRX(8); // Left front (follower)
    rightDrive1 = new WPI_TalonSRX(6); // Right back (master)
    rightDrive2 = new WPI_TalonSRX(5); // Right front (follower)

    intakeSingleSolenoid = new Solenoid(0);
    //intakeDrawerRight = new DoubleSolenoid(2, 3);
    liftDoubleSolenoid = new DoubleSolenoid(1, 2);

    cameraReflectLightSolenoid = new Solenoid(7);
    
    m_exampleSubsystem = new ExampleSubsystem();
    m_NavX = new NavX(ahrs);
    m_IntakeSub = new IntakeSub(intakeMotor, intakeSingleSolenoid);
    m_ShooterSub = new ShooterSub(shooterMotor1, shooterMotor2);
    m_LiftSub = new LiftSub(liftMotor1, liftMotor2, liftDoubleSolenoid);
    m_CameraReflectLight = new CameraReflectLight(cameraReflectLightSolenoid);
    m_FeederSub = new FeederSub(feederMotor);
    //m_Falcon500Sub = new Falcon500Sub(falcon500);
     
    m_DriveTrain = new DriveTrain(leftDrive1, leftDrive2, rightDrive1, rightDrive2, m_NavX);
    m_CameraSub = new CameraSub(camera1, camera2, camera3);

    m_IntakeTestCmd = new IntakeTestCommand(m_IntakeSub, DriverControl, driveButton2, driveButton3);
    m_LiftCmd = new LiftCommand(m_LiftSub, DriverControl, opButton5, opButton6, opButton1, opButton4);

    m_ShootCommand = new Shoot(m_ShooterSub, DriverControl);
    m_FeedCommand = new FeedCommand(m_FeederSub, driveButton5, driveButton6);
    m_autoCommand = new ExampleCommand(m_exampleSubsystem);
    
    m_CameraLightCmd = new CameraLightCommand(m_CameraReflectLight, opButton2);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link DriverController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //driveButton1.whenPressed(m_LiftCmd);
    driveButton2.whenPressed(m_IntakeTestCmd); //driveButton2 and driveButton3 move the intake solenoid in opposite directions
    driveButton3.whenPressed(m_IntakeTestCmd);
    //driveButton4.whenPressed(m_LiftCmd);
    driveButton5.whileHeld(m_FeedCommand); //driveButton5 and driveButton6 move the buffer (feeder) in opposite directions
    driveButton6.whileHeld(m_FeedCommand);

    opButton1.whenPressed(m_LiftCmd); //lift solenoid release
    opButton4.whenPressed(m_LiftCmd); //lift solenoid retract
    opButton5.whileHeld(m_LiftCmd); //lift winch loosen
    opButton6.whileHeld(m_LiftCmd); //lift winch tighten

    opButton2.whenPressed(m_CameraLightCmd);
  }

  public void configDashboard() {
    // To do: need a flag, for example, 'Test Mode', only when it is turned on, show various 
    // measures. Showing these measures on SmartDashBoard will cause loop time of 0.02s overrun

    m_ShooterSub.showValues();

    //SmartDashboard.putNumber("Encoder via DIO", encoder.getDistance();//getQuadraturePosition());
    SmartDashboard.putNumber("PDP Bus voltage", m_PDP.getVoltage());
    SmartDashboard.putNumber("PDP Temperature", m_PDP.getTemperature());
    SmartDashboard.putNumber("PDP Total Current", m_PDP.getTotalCurrent());
    SmartDashboard.putNumber("PDP Total Energy", m_PDP.getTotalEnergy());
    SmartDashboard.putNumber("PDP, Channel 8 (shooter 1), current", m_PDP.getCurrent(8));
    SmartDashboard.putNumber("PDP, Channel 9 (shooter 2), current", m_PDP.getCurrent(9));

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
    m_autoChooser.addOption("Auton Side Cargo Panel", new AutonSideCargoPanel());
    m_autoChooser.setDefaultOption("Cross Hab Line (Default Auto)", AutonCrossHabLine.getInstance());
*/
  }

  public void checkJoystick(){
    double driverLeftX = DriverControl.getRawAxis(0);
    double driverLeftY = DriverControl.getRawAxis(1);
    m_DriveTrain.move(-0.7*driverLeftY, 0.7*driverLeftX);

    double driverRightY = DriverControl.getRawAxis(5);
    if(Math.abs(driverRightY) > 0){
      m_ShootCommand.schedule();
    }

    double leftTrigger = DriverControl.getRawAxis(2);
    double rightTrigger = DriverControl.getRawAxis(3);
    
    if(leftTrigger > 0){
      m_IntakeTestCmd.schedule();
    } else if(rightTrigger > 0){
      m_IntakeTestCmd.schedule();
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
