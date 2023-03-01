// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Inputs;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Auton;
import frc.robot.Constants;




/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {   // RobotBase {
  private Command m_autonomousCommand;

  //Thread m_visionThread;

  //private RobotContainer m_robotContainer;


  // Note: according to WPILIB we MUST use Can Port 1 for Rev Power Board.  (FWL-2022/04/12)
  public static PowerDistribution robotPowerPanel = new PowerDistribution(1, ModuleType.kRev);

  /* Power Panel from Rev Robotics has 0-23 ports. 
    We will list as +1 so the first port, port will show as 1
    Java uses 0 offsets so this can be confusing to Muggles.
  */
  private String[] aryPowerPortName = { 
                        "RL Drive",           // 0
                        "RL Turn",
                        "RR Drive",
                        "RR Turn",
                        "FR Turn",
                        "FR Drive",
                        "Turn Encoders",
                        "Small Shooter Roller",
                        "Intake Deploy",
                        "Tony + Intake Roller",
                        "Turret",             // 10
                        "Gib Roller",
                        "Lift 1-stage 1",
                        "Lift 2-hand",
                        "Dart",
                        "Shooter Roller",
                        "Camera",
                        "Switch",
                        "FL Turn",
                        "FL Drive",
                        "Port 20",          // 20
                        "Port 21", 
                        "Port 22",
                        "Port 23" 
                    };

  //public Auton auton = new Auton();
 // public ClimberAutomation climberAutomation = new ClimberAutomation();
  // create public instances for the subsystems
  // we do not need to do this for Inputs as it is already static
  public Auton auton = new Auton();
  public DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
  public IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  public PneumaticSubsystem m_PneumaticSubsystem = new PneumaticSubsystem();
  public TowerSubsystem m_TowerSubsystem = new TowerSubsystem();
  public VisionSubsystem m_VisionSubsystem = new VisionSubsystem();
  
  //public static PlayBack pbSquare = new PlayBack( "/c/1218Data/playback", "playback_square.csv" );
  //public static PlayBack pbAuton3ball = new PlayBack( "/c/1218Data/playback", "auton_3_ball_playback.csv" );
  //public static PlayBack pbAutonTest = new PlayBack( "/c/1218Data/playback", "autontest_playback.csv" );

  private static boolean stickyPowerAlarmsReset = false;
  


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    //CameraServer.startAutomaticCapture();

    //m_robotContainer = new RobotContainer();
    //m_robotContainer = new RobotContainer();

    /*/
    m_visionThread =
        new Thread(
            () -> {
              // Get the UsbCamera from CameraServer
              UsbCamera camera = CameraServer.startAutomaticCapture();
              // Set the resolution
              camera.setResolution(640, 480);

              // Get a CvSink. This will capture Mats from the camera
              CvSink cvSink = CameraServer.getVideo();
              // Setup a CvSource. This will send images back to the Dashboard
              CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

              // Mats are very memory expensive. Lets reuse this Mat.
              Mat mat = new Mat();

              // This cannot be 'true'. The program will never exit if it is. This
              // lets the robot stop this thread when restarting robot code or
              // deploying.
              while (!Thread.interrupted()) {
                // Tell the CvSink to grab a frame from the camera and put it
                // in the source mat.  If there is an error notify the output.
                if (cvSink.grabFrame(mat) == 0) {
                  // Send the output the error.
                  outputStream.notifyError(cvSink.getError());
                  // skip the rest of the current iteration
                  continue;
                }
                // Put a rectangle on the image
                Imgproc.rectangle(
                    mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
                // Give the output stream a new image to display
                outputStream.putFrame(mat);
              }
            });
    m_visionThread.setDaemon(true);
    m_visionThread.start();
    */

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    
    //CommandScheduler.getInstance().run();
    //CommandScheduler.getInstance().run();
    // unlike the command scheduler this will shot the order we will run

  }

  // Initialization code for disabled mode should go here.
  @Override
  public void disabledPeriodic(){
    Inputs.periodic();
    SmartDashboard.putNumber("Auton to run", Inputs.autonToRun);
      // Constants.telemetry.saveSpreadSheet();
      //Constants.telemetry.save

  }


  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    //auton.iAutonId = 1;
    setAllianceInfo(); // get alliance color from driver station 
    auton.reset();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    Inputs.periodic();                // Make inputs read all their variables

    // Auton code goes here
    auton.executeAuton(Inputs.autonToRun);

    // process each subsystem in order now that we have inputs read in
    callRobotSubsystems();      // call other robot processes in order


  }

  @Override
  public void teleopInit() {

    setAllianceInfo(); // get alliance color from driver station 

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    //if (m_autonomousCommand != null) {
    // m_autonomousCommand.cancel();
    //
  
    //Inputs.driveResetGyro = true;

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    
    Inputs.periodic();             // Make inputs read all their variables

    // process each subsystem in order now that we have inputs read in
    callRobotSubsystems();      // call other robot processes in order

   }

  @Override
  public void testInit() {

      setAllianceInfo(); // get alliance color from driver station 
      auton.reset();

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic(){

    Inputs.periodic();                // Make inputs read all their variables

    // Auton code goes here
    auton.executeAuton(9);    // 9 is set asside for testing. 

    // process each subsystem in order now that we have inputs read in
    callRobotSubsystems();      // call other robot processes in order

  }

  private void callRobotSubsystems(){      // call other robot processes in order

    // Inputs.saveTelem();               // do this here so we capture any changes made by auton. 

    // m_VisionSubsystem.periodic();     // process vision targgets do it here so subs can use it.
    m_TowerSubsystem.periodic(); 
    m_IntakeSubsystem.periodic();
    m_PneumaticSubsystem.periodic();
    m_DriveSubsystem.periodic();      // two steps,  here determine where we are. 

    // YK - Disabled telemetry 2023-02-27
    // trackRobotBattery();              // write out telemetry fore the battery ports
    // Constants.telemetry.writeRow();   // after all the susbsystems run write telem row. 
  }

  private void trackRobotBattery() {

    /*
    String key = "";

    Constants.telemetry.saveDouble( "Robot Total Current", robotPowerPanel.getTotalCurrent() );
    Constants.telemetry.saveDouble( "Robot Total Power", robotPowerPanel.getTotalPower() );
    Constants.telemetry.saveDouble( "Robot Total Energy", robotPowerPanel.getTotalEnergy() );
    Constants.telemetry.saveDouble( "Robot Total Volts", robotPowerPanel.getVoltage() );
    //Constants.telemetry.saveDouble( "Robot Panel Temp", robotPowerPanel.getTemperature() );

    for( int portId = 0; portId < 24; portId++){
      key = "Robot Power Port " + String.format("%02d", portId) + " " + aryPowerPortName[portId];
      Constants.telemetry.saveDouble(key, robotPowerPanel.getCurrent(portId));  
    }
    */
  }

  private void setAllianceInfo(){

    Constants.MatchSettings.kAllianceColorName = DriverStation.getAlliance().toString().strip().toLowerCase();

    if( Constants.MatchSettings.kAllianceColorName.startsWith("b") )           // blue string?
        Constants.MatchSettings.kAllianceColor = Constants.MatchSettings.kBlueAlliance;
    else if( Constants.MatchSettings.kAllianceColorName.startsWith("r"))       // red string?
        Constants.MatchSettings.kAllianceColor = Constants.MatchSettings.kRedAlliance;
    else 
        Constants.MatchSettings.kAllianceColor = 0;    // not set
  }
}

