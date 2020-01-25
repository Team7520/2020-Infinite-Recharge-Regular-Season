/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team7520.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
//import edu.wpi.first.wpilibj.templates.commands.CommandBase;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GamepadBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.DigitalInput;

// import edu.wpi.first.wpilibj.VictorSP;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import frc.team7520.robot.commands.*;
import frc.team7520.robot.subsystems.*;
import frc.team7520.robot.sensors.*;
import frc.team7520.robot.DriveControls;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static DriveTrain m_driveTrain = new DriveTrain();
  public static ArmSub m_subArm = new ArmSub();
  public static FrontClimbSub m_subFrontClimb = new FrontClimbSub();
  public static HatchSub m_subHatch = new HatchSub();
  public static ConveyerSub m_subConveyer = new ConveyerSub();
  public static CollectBarsSub m_subCollectBars = new CollectBarsSub();

  public static OI m_oi;

  public static NavX navX = new NavX(new AHRS(SPI.Port.kMXP));
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
//public static exampleSolenoid m_subExampleSolenoid = new exampleSolenoid();


//driver Joysticks
//private Joystick OneJoystick; //dri ver
//private Joystick OtherJoystick; //oper ator

//private gamepad

private Joystick XboxControl;
private Joystick driverController;

//private Joystick gamepad1;

//speed controllers for Drive
//private VictorSPX leftDrive1;
WPI_TalonSRX leftDrive1;
WPI_VictorSPX leftDrive2;
//private VictorSPX rightDrive1;
WPI_TalonSRX rightDrive1;
WPI_VictorSPX rightDrive2;

public VictorSPX ballIntake;

public VictorSPX armMotor;

public VictorSPX hatchOuttake;

public VictorSPX climbReturn;

// Ryerson event- our climbing doesn't work anyway, 
//so these motors are disabled
//public VictorSPX leftClimber;
//public VictorSPX rightClimber;

public VictorSPX leftCollectBar;
public VictorSPX rightCollectBar;

private long autoStartTime;

private UsbCamera cameraFront;
private UsbCamera cameraRear;
private UsbCamera cameraArm;

private boolean updateSensorsStatus = false;

//private Solenoid exampleSolenoid1;
public DoubleSolenoid exampleSolenoid1;

public Button buttoMoveStraight;
public Button button1;
public Button button2;
public Button button3;
public Button button4;
public Button button5;
public Button button6;
public Button button7;
public Button button8;
DifferentialDrive drive;
DigitalInput upArmLimitSwitch, upCollectBarLimitSwitch, intakeLowerLimitSwitch;
public boolean whichControls = false;
public boolean intakeStop = false;
public boolean prevIntakeLowerLimitSwitch; //the previous value of intakeLowerLimitSwitch 
public boolean stopMoving = false; //flag that tells certain components to stop moving

public CargoIntake cargoIntakeCmd;
public hatchIn hatchInCmd;
public hatchOut hatchOutCmd;


public LimitSensors limitSensors = new LimitSensors();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    SmartDashboard.putString("Auton Chooser", " select before each game");
    m_oi = new OI();
    m_chooser.setDefaultOption("Hatch In (up) (Default Auton)", new hatchIn());
    m_chooser.addOption("Hatch Out (down)", new hatchOut());
    m_chooser.addOption("Collect Bar Up", new CollectBarRaising());
    m_chooser.addOption("Collect Bar Down", new CollectBarLowering());
    m_chooser.addOption("Cargo Intake", new CargoIntake());
    m_chooser.addOption("Cargo Shoot", new CargoShoot());
    m_chooser.addOption("Arm Up To Limit", new ArmToUpLimit());
    m_chooser.addOption("Hatch In and Out",new CommandGroupTest()); //testing commandgroups

    m_chooser.addOption("Cross Hab Line (Default Auto)", new AutonCrossHabLine());
    //m_chooser.addOption("Do Nothing (idle)", new AutonDoNothing());
    //m_chooser.addOption("Front Cargo Panel", new AutonFrontCargoPanel());
    //m_chooser.addOption("Side Cargo Panel", new AutonSideCargoPanel());
    //m_chooser.addOption("Rocket Cargo Panel", new AutonRocketPanel());
    //m_chooser.addOption(name, object);

    //m_chooser.addObject("Ed's Controls", DriveControls.ED_CONTROLS);
    SmartDashboard.putBoolean("Ed's Controls", true);
    
    
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);

    SmartDashboard.getBoolean("Update sensors status", updateSensorsStatus);
    System.out.println("robotInit");

 //this.OtherJoystick = new Joystick(0);
 //this.OneJoystick = new Joystick(1); 

 this.XboxControl = new Joystick(2);
 driverController = new Joystick(1);


 //buttoMoveStraight = new JoystickButton(this.OtherJoystick, 1);
 button1 = new JoystickButton(XboxControl, 1);
 button2 = new JoystickButton(XboxControl, 2);
 button3 = new JoystickButton(XboxControl, 3);
 button4 = new JoystickButton(XboxControl, 4);
 button5 = new JoystickButton(XboxControl, 5);
 button6 = new JoystickButton(XboxControl, 6);
 button7 = new JoystickButton(XboxControl, 7);
 button8 = new JoystickButton(XboxControl, 8);

 //otherButton1 = new JoystickButton(OtherJoystick, 1);

 //limitSensors.init();

 upArmLimitSwitch = new DigitalInput(2);
 upCollectBarLimitSwitch = new DigitalInput(1);
 intakeLowerLimitSwitch = new DigitalInput(3);
 prevIntakeLowerLimitSwitch = intakeLowerLimitSwitch.get();

 
 
 /* 01/13/19- code deploys successfully but still
can't control the robot with joysticks- can try
disabling camera stuff to see if it works*/
 /*try {
   this.cameraFront = new UsbCamera("usb cam", "/note/video0");
   CameraServer.getInstance().addCamera(this.cameraFront);
//   this.cameraFront.setResolution(320, 240);
   this.cameraFront.setResolution(160, 120);
//   this.cameraFront.setFPS(30);
   this.cameraFront.setFPS(10);
 } catch (Exception e) {
   e.printStackTrace();
 }

 try {
  this.cameraRear = new UsbCamera("usb cam rear", "/note/video1");
  CameraServer.getInstance().addCamera(this.cameraRear);
  this.cameraRear.setResolution(160, 120);
  this.cameraRear.setFPS(10);
} catch (Exception e) {
  e.printStackTrace();
}
 */

//CameraServer.getInstance().startAutomaticCapture();
cameraFront = CameraServer.getInstance().startAutomaticCapture("Front Cam", 2);
//cameraFront.setResolution(320, 240);
//set to low resolution to save bandwidth
cameraFront.setResolution(160, 120);
// cameraFront.setFPS(30);
cameraFront.setFPS(10);

cameraRear = CameraServer.getInstance().startAutomaticCapture("Rear Cam", 0);
//set to low resolution to save bandwidth
cameraRear.setResolution(160, 120);
// cameraFront.setFPS(30);
cameraRear.setFPS(10);

cameraArm = CameraServer.getInstance().startAutomaticCapture("Arm Cam", 1);
//set to low resolution to save bandwidth
cameraArm.setResolution(160, 120);
// cameraFront.setFPS(30);
cameraArm.setFPS(10);

  this.leftDrive1 = new WPI_TalonSRX(0);
  this.leftDrive2 = new WPI_VictorSPX(1);

  this.rightDrive1 = new WPI_TalonSRX(2);
  this.rightDrive2 = new WPI_VictorSPX(3);

  this.ballIntake = new VictorSPX(4);

  this.armMotor = new VictorSPX(5);

  this.hatchOuttake = new VictorSPX(6);

  this.climbReturn = new VictorSPX(7);

  //Ryerson event- our climbing doesn't work anyway : 
  //this.leftClimber = new VictorSPX(8);
  //this.rightClimber = new VictorSPX(9);

  this.leftCollectBar = new VictorSPX(8);
  this.rightCollectBar = new VictorSPX(9);
  this.rightCollectBar.setInverted(true);
  // set follower
  this.rightCollectBar.set(ControlMode.Follower , 8);

  // set motors configurations
  setMotorsConfigurations();

  // set followers
  this.leftDrive2.set(ControlMode.Follower , 0);
  this.rightDrive2.set(ControlMode.Follower , 2);

  drive = new DifferentialDrive(leftDrive1, rightDrive1);

//this.ballIntake.set(ControlMode.Follower , 1);

  this.exampleSolenoid1 =  new DoubleSolenoid(0,1); 

  // assign motors for subsystems
  m_driveTrain.setLeftMasterMotor(leftDrive1);
  m_driveTrain.setRightMasterMotor(rightDrive1);
  m_driveTrain.setDriveTrain(leftDrive1, rightDrive1);

  m_subArm.setMotor(this.armMotor);
  m_subConveyer.setMotor(this.ballIntake);
  m_subHatch.setMotor(hatchOuttake);
  m_subCollectBars.setMotor(leftCollectBar); // left is master motor

  SmartDashboard.putString("Front Camera", "intake");
  SmartDashboard.putString("Rear Camera", "shooter");
  SmartDashboard.putString("Arm Camera" , "arm");


  hatchInCmd = new hatchIn(); //the commands here are not scheduled to run for some reason
  hatchOutCmd = new hatchOut(); //these commands are put under a "value" parameter; do these commands give values?
  cargoIntakeCmd = new CargoIntake();

  // sensor reading on/off = 
  SmartDashboard.putBoolean("Update sensors status", upArmLimitSwitch.get());
  showSensorsStatus();
  // test commands
  // SmartDashboard.putString("Test Commands", "Press start to test a command");
  // SmartDashboard.putData("FrontArmPushDownForL2", new FrontArmPushDownForL2());
  // SmartDashboard.putData("Collect Cargo Using Collect Bars", new CollectCargoCmdGrp());
  // SmartDashboard.putData("Collect bar to upper limit", new CollectBarToUpLimit());
  // SmartDashboard.putData("Cargo Intake", cargoIntakeCmd);
  // SmartDashboard.putData("Cargo Shoot", new CargoShoot());
  // SmartDashboard.putData("Hatch in",   new hatchIn()); //the commands here are not scheduled to run for some reason
  // SmartDashboard.putData("Hatch out", new hatchOut()); //these commands are put under a "value" parameter; do these commands give values?
  // SmartDashboard.putData("Hatch stop", new hatchStop());
  // SmartDashboard.putData("Piston in", new pistonIn());
  // SmartDashboard.putData("Piston out", new pistonOut());
  SmartDashboard.updateValues();

  //CommandBase.init();
  }

  private void setMotorsConfigurations() { 
  //    talonRightBack.configContinuousCurrentLimit(20, 0);
        
    this.leftDrive1.enableCurrentLimit(true);
    this.leftDrive1.configContinuousCurrentLimit(20, 0); 

//    this.leftDrive2.enable.enableCurrentLimit(true);// not available for a Victor motor controller
//    this.leftDrive2.configContinuousCurrentLimit(20, 0);// not available for a Victor motor controller

    this.rightDrive1.enableCurrentLimit(true);
    this.rightDrive1.configContinuousCurrentLimit(20, 0);

//    this.rightDrive2.enableCurrentLimit(true);// not available for a Victor motor controller
//    this.rightDrive2.configContinuousCurrentLimit(20, 0);// not available for a Victor motor controller

    //applying ramp ups to motors
    
    this.leftDrive1.configOpenloopRamp(0.10, 0);
    this.leftDrive2.configOpenloopRamp(0.10, 0);
    this.rightDrive1.configOpenloopRamp(0.10, 0);
    this.rightDrive2.configOpenloopRamp(0.10, 0);

    this.armMotor.configOpenloopRamp(0.0, 0);
    

    this.armMotor.setNeutralMode(NeutralMode.Brake);
    this.hatchOuttake.setNeutralMode(NeutralMode.Brake); 
  
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    updateSensorsStatus = SmartDashboard.getBoolean("Update sensors status", false);
    if(updateSensorsStatus == true)
      showSensorsStatus();
  
  }

  private void showSensorsStatus()
  {
    SmartDashboard.putBoolean("Arm Up", upArmLimitSwitch.get());
    SmartDashboard.putBoolean("CollectBar Up", upCollectBarLimitSwitch.get());
    SmartDashboard.putBoolean("Lower Intake", intakeLowerLimitSwitch.get());

    SmartDashboard.putNumber("NavX Yaw  (+ CW, - CCW)",    navX.getZAngle());
    SmartDashboard.putNumber("NavX Pitch(+ Tilt Backward)", navX.getPitch());
    SmartDashboard.putNumber("NavX Roll (+ Roll Left)", navX.getRoll());
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    this.autoStartTime = System.currentTimeMillis();

    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    //Scheduler.getInstance().add(new CommandGroupTest());
    //Scheduler.getInstance().add(new hatchIn()); //runs multiple times
/*
   long timePassed = System.currentTimeMillis() - this.autoStartTime;
   
   SmartDashboard.putNumber("Time Passed" , (double)timePassed);
   if(timePassed < 3000){ 
    this.leftDrive1.set(ControlMode.PercentOutput , -0.3 );
   // this.leftDrive2.set(ControlMode.PercentOutput , -0.3);
    this.rightDrive1.set(ControlMode.PercentOutput , 0.3);
  //  this.rightDrive2.set(ControlMode.SPercentOutput , 0.3);
   
  }

   else if(timePassed < 5000)  {//turn motors same way to turn the robot
   this.leftDrive1.set(ControlMode.PercentOutput ,0.3);
   //this.leftDrive2.set(ControlMode.PercentOutput , 0.3);
   this.rightDrive1.set(ControlMode.PercentOutput , 0.3);
  // this.rightDrive2.set(ControlMode.PercentOutput , 0.3);
   } 
   else {
    this.leftDrive1.set(ControlMode.PercentOutput , 0.0);
   // this.leftDrive2.set(0.0);
    this.rightDrive1.set(ControlMode.PercentOutput , 0.0);
   // this.rightDrive2.set(0.0);
   }
*/

//teleopGamePadControl();

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
    Scheduler.getInstance().run();

    teleopGamePadControl();

  }

  public void teleopGamePadControl() {

    //05/19/2019 - programming objectives for the summer: -working on having usable commands/auton
    //-vision using opencv- recognize cargo balls at the very least

    double driverLeftX = driverController.getRawAxis(0);
    double driverLeftY = driverController.getRawAxis(1);
    double driverRightX = driverController.getRawAxis(4);
    double driverRightY = driverController.getRawAxis(5);

    if(whichControls == true) {

        drive.arcadeDrive(driverLeftY, driverLeftX); 
        //should be stick forward = robot backward, stick backward = robot forward
        
    }

    else {
        drive.arcadeDrive(-driverLeftY, driverLeftX);
        //should be stick forward = robot forward, stick backward = robot backward
        
    }

   

    /* 
    timedRobot - uses a timer (Notifier) to guarantee that 
    the periodic methods are called at a predictable time 
    interval
    */
    //System.out.println("Ok");
    //System.out.println("teleopPeriodic");
    // tank style drive control


    // getting our inputs
    // double leftStick = this.driver.getRawAxis(0);
    // double rightStick = this.driver.getRawAxis(1);

    /*  double leftStick = this.operator.getRawAxis(0);
        double rightStick = this.operator.getRawAxis(1);

    // calculating our outputs


    //setting speed controllers
    this.leftDrive1.set(-leftStick);
    this.leftDrive2.set(-leftStick);

    this.rightDrive1.set(rightStick);
    this.rightDrive2.set(rightStick);
    */

    // arcade drive 

    // inputs from the driver 
    //double operatorX = this.operator.getRawAxis(0);
    //double operatorY = this.OtherJoystick.getRawAxis(1);

    //double driverX = this.driver.getRawAxis(0);
    // double driverY = this.OneJoystick.getRawAxis(1);
    // double rotation = this.OneJoystick.getRawAxis(2);// rotates_the_bot
    double gamepadY = -this.XboxControl.getRawAxis(5); //1

    double gamepad1Y = this.XboxControl.getRawAxis(1);//5
    
    //System.out.println(driverX+','+driverY);
    // calculations of output
    // double leftOut = operatorY; 
    //double rightOut = driverY;

    double armOut = gamepadY;

    double intakeOut = gamepad1Y;
    

    double minDriveTrainOut = 0.1;
    /*if(Math.abs(leftOut) < minDriveTrainOut)
      leftOut = 0;

    if(Math.abs(rightOut) < minDriveTrainOut)
      rightOut = 0;*/
    

    // outputs to speed controllers
    
    //this.leftDrive1.set(ControlMode.PercentOutput , (-leftOut));
    //this.leftDrive2.set(ControlMode.PercentOutput , -leftOut);
    // this.rightDrive1.set(ControlMode.PercentOutput , (rightOut));
    //this.rightDrive2.set(ControlMode.PercentOutput , rightOut);
    // buttoMoveStraight.whenPressed(command);
    //m_driveTrain.move((-leftOut), (rightOut));

    if(Math.abs(armOut) > 0.05){
      if((armOut > 0 && upArmLimitSwitch.get()) || armOut < 0)
        this.armMotor.set(ControlMode.PercentOutput , (-armOut)); //to make the intake less sensitive
    }
    else 
      this.armMotor.set(ControlMode.PercentOutput , 0);

    
    

    if((Math.abs(intakeOut) > 0.1))  { 
      this.ballIntake.set(ControlMode.PercentOutput , (intakeOut * 0.75));
/*      limit switch stuff that was written during the ON provincials
// intakeLowerLimitSwitch.get()
      // logic was disscussed after the 1st day of matches at the ON provincials Science division
      if(intakeLowerLimitSwitch.get()) {
        this.ballIntake.set(ControlMode.PercentOutput , (intakeOut * 0.75));
      }
      else if(!intakeLowerLimitSwitch.get() && !prevIntakeLowerLimitSwitch) {
        // the driver has to release intake control buttons
        if(stopMoving) {
          this.ballIntake.set(ControlMode.PercentOutput , 0); //to make the intake less sensitive
        } else {
          this.ballIntake.set(ControlMode.PercentOutput , (intakeOut * 0.75));
        }
        prevIntakeLowerLimitSwitch = intakeLowerLimitSwitch.get();
      } else if(!intakeLowerLimitSwitch.get()) //&& prevIntakeLowerLimitSwitch)
       {
          this.ballIntake.set(ControlMode.PercentOutput , 0);
          stopMoving = true;
          prevIntakeLowerLimitSwitch = intakeLowerLimitSwitch.get();
        }
        */ 
      } else {
        stopMoving = false;
        this.ballIntake.set(ControlMode.PercentOutput , 0);
        prevIntakeLowerLimitSwitch = intakeLowerLimitSwitch.get();
    }  

    if(button2.get())
    {
      this.exampleSolenoid1.set(DoubleSolenoid.Value.kForward);
    }
    if(button3.get())
    {
      this.exampleSolenoid1.set(DoubleSolenoid.Value.kReverse);
    }
    
//    button1.whenPressed(new hatchIn());
//    button4.whenPressed(new hatchOut());
    
    if(!m_subHatch.isUsed) // if it is not being used by a command right now
    {
      if(!button1.get() && !button4.get())
      {
        this.hatchOuttake.set(ControlMode.PercentOutput , 0);
      }
      else if(button1.get())
      {
        //this.hatchOuttake.set(ControlMode.PercentOutput , 1);
        m_subHatch.hatchIn();
      }
      else if(button4.get())
      {
        //this.hatchOuttake.set(ControlMode.PercentOutput , -1);
        m_subHatch.hatchOut();
      }
    }

   /* if(otherButton1.get()) //this allows for straight driving
    {
    this.leftDrive1.set(ControlMode.PercentOutput , (-leftOut));
    this.rightDrive1.set(ControlMode.PercentOutput , (leftOut));
    }*/

 /* Ryerson event- our climbing doesn't work anyway : (   
 // buttons 5(left) and 6(right) are to be used for climbing
    if(button5.get() || button7.get()) { 
        if(button5.get()) {
          this.leftClimber.set(ControlMode.PercentOutput, -1);
        }
        else if(button7.get()) {
          this.leftClimber.set(ControlMode.PercentOutput, 1);
        }        
    }
    else {
      this.leftClimber.set(ControlMode.PercentOutput, 0);
    }

  if(button6.get() || button8.get()) { 
    if(button6.get()) {
      this.rightClimber.set(ControlMode.PercentOutput, 1);
    }
    else if(button8.get()) {
      this.rightClimber.set(ControlMode.PercentOutput, -1);
    }        
  }
  else {
    this.rightClimber.set(ControlMode.PercentOutput, 0);
  }
    
  if((XboxControl.getPOV() == 180) || (XboxControl.getPOV() == 0)) {
    if(XboxControl.getPOV() == 180) {
      this.climbReturn.set(ControlMode.PercentOutput, 1);
    }
    else if(XboxControl.getPOV() == 0) {
      this.climbReturn.set(ControlMode.PercentOutput, -1);
    }
  }
  else {
    this.climbReturn.set(ControlMode.PercentOutput, 0);
  }
  */ 

   // buttons 5(left) and 6(right) are to be used for ball collect bars
   // note: leftCollectBar motor is master,right one is follower  
  if(button5.get() || button6.get()) { 
    if(button5.get() && upCollectBarLimitSwitch.get()) {
      this.leftCollectBar.set(ControlMode.PercentOutput, 0.8);
    }
    else if(button6.get()) {
        this.leftCollectBar.set(ControlMode.PercentOutput, -0.8);
      }        
    }
    else {
        this.leftCollectBar.set(ControlMode.PercentOutput, 0);
    }



//      button2.whenPressed(new pistonIn());

//      button3.whenPressed(new hatchIn());
/*
      button4.whenPressed(new hatchOut());

      button3.whenReleased(new hatchStop());
      button4.whenReleased(new hatchStop());
      
  */
  }
/**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }


}
