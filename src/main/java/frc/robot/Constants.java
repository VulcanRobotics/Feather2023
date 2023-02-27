// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

  
public final class Constants {
  public static final Config config           = new Config("/c/1218Data", "1218Config.txt");
  public static final SCHTelemetry telemetry  = new SCHTelemetry("/c/1218Data", "1218_Telemetry", "Telemetry_Columns.txt");    // do not add extension, code will add it. 
  // when ready please call Frank Larkin at 215-823-9593
  public static final String whichRobot  = "Heather";
  public static final class Display {

    public static final double kKLEDDefaultDisplay    =  0.49;
    public static final double kKLEDOnTargetDisplay   = -0.07;
    public static final double kKLEDTargettingDisplay = -0.09;


  }

  public static final class Swerve {
    public static final double driveGearRatio = (6.86 / 1.0); //6.86:1
    public static final double angleGearRatio = 32 / 3;//12.8;//10.67;

    public static final double trackWidth = 0.457;
    public static final double wheelBase = 0.457;
    public static final double wheelDiameter = Units.inchesToMeters(3.94);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double turnWheel360Ticks = 31597.3632;
    public static final double driveWheel360Ticks = 2048.0 * driveGearRatio;
  

    public static final double ticksPerMeter = driveWheel360Ticks/wheelCircumference;

    public static final double maxSpeed = 4.5; //meters per second
    public static final double maxAngularVelocity = 11.5;
  }

  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort    = config.getInt("SWRV_DRV_kFrontLeftDriveMotorPort",21); 
    public static final int kRearLeftDriveMotorPort     = config.getInt("SWRV_DRV_kRearLeftDriveMotorPort", 23 ); 
    public static final int kFrontRightDriveMotorPort   = config.getInt("SWRV_DRV_kFrontRightDriveMotorPort", 22 ); 
    public static final int kRearRightDriveMotorPort    = config.getInt("SWRV_DRV_kRearRightDriveMotorPort", 24 ); 

    public static final int kFrontLeftTurningMotorPort  = config.getInt("SWRV_DRV_kFrontLeftTurningMotorPort", 25 );
    public static final int kRearLeftTurningMotorPort = config.getInt("SWRV_DRV_kRearLeftTurningMotorPort", 27);
    public static final int kFrontRightTurningMotorPort = config.getInt("SWRV_DRV_kFrontRightTurningMotorPort", 26);
    public static final int kRearRightTurningMotorPort = config.getInt("SWRV_DRV_kRearRightTurningMotorPort", 28);

    public static final int kFrontLeftTurningEncoderPorts = config.getInt("SWRV_DRV_kFrontLeftTurningEncoderPorts", 35);
    public static final int kRearLeftTurningEncoderPorts = config.getInt("SWRV_DRV_kRearLeftTurningEncoderPorts", 37);
    public static final int kFrontRightTurningEncoderPorts = config.getInt("SWRV_DRV_kFrontRightTurningEncoderPorts", 36);
    public static final int kRearRightTurningEncoderPorts = config.getInt("SWRV_DRV_kRearRightTurningEncoderPorts", 38);

    public static final boolean kFrontLeftTurningEncoderReversed = !false;           // nailed up, no config
    public static final boolean kRearLeftTurningEncoderReversed = !false;
    public static final boolean kFrontRightTurningEncoderReversed = !false;
    public static final boolean kRearRightTurningEncoderReversed = !false;

    public static final int kFrontLeftDriveEncoderPorts = config.getInt("SWRV_DRV_kFrontLeftDriveMotorPort",21);
    public static final int kRearLeftDriveEncoderPorts = config.getInt("SWRV_DRV_kRearLeftDriveMotorPort", 23);
    public static final int kFrontRightDriveEncoderPorts = config.getInt("SWRV_DRV_kFrontRightDriveMotorPort", 22);
    public static final int kRearRightDriveEncoderPorts = config.getInt("SWRV_DRV_kRearRightDriveMotorPort", 24);

    public static final boolean kFrontLeftDriveEncoderReversed = false;//false  // no config, nailed //true 1/14
    public static final boolean kRearLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = !false;
    public static final boolean kRearRightDriveEncoderReversed = !true; //true //fale 1/14

    public static final double kFrontLeftTurnEncoderOffsetDeg = 214.1; //config.getDouble("SWRV_DRV_kFrontLeftTurnEncoderOffsetDeg", 212); //180- Math.toDegrees(-0.278); //More Positive  
    public static final double kFrontRightTurnEncoderOffsetDeg = 18.5;//config.getDouble("SWRV_DRV_kFrontRightTurnEncoderOffsetDeg", 23); //360-Math.toDegrees(1.315);
    public static final double kRearLeftTurnEncoderOffsetDeg =  -113; //config.getDouble("SWRV_DRV_kRearLeftTurnEncoderOffsetDeg", -111); //180-Math.toDegrees(-0.629);
    public static final double kRearRightTurnEncoderOffsetDeg = -17.5; //config.getDouble("SWRV_DRV_kRearRightTurnEncoderOffsetDeg", 5.0); // fwl - update 3/4/22 //360-Math.toDegrees(-1.459);

    public static final double kPerseveranceFrontLeftTurnEncoderOffsetDeg = -159.8;//214.1; //config.getDouble("SWRV_DRV_kFrontLeftTurnEncoderOffsetDeg", 212); //180- Math.toDegrees(-0.278); //More Positive  
    public static final double kPerseveranceFrontRightTurnEncoderOffsetDeg = -210.3; //18.5;//config.getDouble("SWRV_DRV_kFrontRightTurnEncoderOffsetDeg", 23); //360-Math.toDegrees(1.315);
    public static final double kPerseveranceRearLeftTurnEncoderOffsetDeg = -76.4; //-113; //config.getDouble("SWRV_DRV_kRearLeftTurnEncoderOffsetDeg", -111); //180-Math.toDegrees(-0.629);
    public static final double kPerseveranceRearRightTurnEncoderOffsetDeg = -305.5;//-17.5; //config.getDouble("SWRV_DRV_kRearRightTurnEncoderOffsetDeg", 5.0); // fwl - update 3/4/22 //360-Math.toDegrees(-1.459);

    public static final double kHeatherFrontLeftTurnEncoderOffsetDeg = 214.1;
    public static final double kHeatherFrontRightTurnEncoderOffsetDeg = 18.5;
    public static final double kHeatherRearLeftTurnEncoderOffsetDeg = -113.0;
    public static final double kHeatherRearRightTurnEncoderOffsetDeg = -17.5;
    //sensor coefficent = .0015339775

    public static final double kJoystickDeadband = config.getDouble("INPUT_JoystickDeadband", 0.10);    // 0.09

    public static final double kTrackWidth = .457;//0.565;  // PGR 30" square perimeter  // no config as this is nailed up
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = .457;//0.565;         //cange                       // no config as this is nailed up
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final double ksVolts = 1.0;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;
    // not used in code 

    public static final double kMaxSpeedMetersPerSecond = config.getDouble("SWRV_SPEED_kMaxSpeedMetersPerSecond", 3.0);
    public static final double kMaxAngularSpeedRadiansPerSecond = config.getDouble("SWRV_SPEED_kMaxAngularSpeedRadiansPerSecond", 3.0);//1.5;//5; 


    public static final double kAdjustment = config.getDouble("SWRV_kAdjustment", 0.001);
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = config.getDouble("SWRV_kMaxModuleAngularSpeedRadiansPerSecond", 2 * Math.PI);
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 4.5;//4.5//config.getDouble("SWRV_kMaxModuleAngularAccelerationRadiansPerSecondSquared", 2 * Math.PI);

    public static final double kTurnPIDTolerance = config.getDouble("SWRV_PID_TurnPIDTolerance", 0.005 );

    public static final int kEncoderCPR = 4096;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kDriveEncoderDistancePerPulse = 
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR / 6.25;

    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) kEncoderCPR;

    /*public static final double kPModuleTurningController = config.getDouble("SWRV_PID_TurningController_KP", 0.65); //0.5
    public static final double kDModuleTurningController = config.getDouble("SWRV_PID_TurningController_KD", 0.0); //0.01; //0.01;

    public static final double kPModuleDriveController = config.getDouble("SWRV_PID_DriveController_KP", 1);   // no confgi nailed up
    public static final double kDModuleDriveController = config.getDouble("SWRV_PID_DriveController_KD", 0.1);   // no confgi nailed up*/


    //NOTE: MAINTAIN THESE PID VALUES
    public static final double kPModuleTurningController = config.getDouble("SWRV_PID_kPModuleTurningController", 0.5); 
    public static final double kDModuleTurningController = config.getDouble("SWRV_PID_kDModuleTurningController", 0.0); 

    public static final double kPModuleDriveController = 1.0;   // no confgi nailed up
    public static final double kDModuleDriveController = 0.0;

    public static final double kDriveVelocityTolerance = config.getDouble("SWRV_PID_DrivePIDTolerance", 0.01);  //PGR 2021

    public static final double kCorrectionTargetWidth = config.getDouble("SWRV_CORRECT_TargetWidth", 1.0);
    public static final double kCorrectionWindow = config.getDouble("SWRV_CORRECT_Window", 10.00);  
    public static final double kCorrectionProportion = config.getDouble("SWRV_CORRECT_Proportion", 0.05); 
    public static final double kCorrectionMinReponse = config.getDouble("SWRV_CORRECT_MinResponse", 0.075); 
    public static final double kCorrectionMaxReponse = config.getDouble("SWRV_CORRECT_MaxResponse", 0.15);  

    public static final double kHeatherTalon360 = 24576.0;
    public static final double kPerseverance360 = 31604.76;

    public static final double kHeatherDegreesToTicks = 68.267;
    public static final double kPerseveranceDegreesToTicks = 87.791;
  
  
  
  }

  public static final class OIConstants {
    /*public static final int kDriverControllerPort = config.getInt("INPUT_kDriverControllerPort", 0);
    public static final int kJoystickPort = config.getInt("INPUT_kJoystickPort", 1);*/
    public static final int kDriverControllerPort = config.getInt("INPUT_kDriverControllerPort", 0);
    public static final int kOperatorControllerPort = config.getInt("INPUT_kOperatorControllerPort", 1);
    public static final int kExtraBoxPort = config.getInt("INPUT_kExtraBoxPort", 3);
    
    // these can be lowered to allow kids to drive the robot. 1.0 is full power
    public static final double kDriverPowerPCT = config.getDouble("INPUT_kDriverPowerPCT", 0.7);    //1.0);   
    public static final double kDriverTurnPCT = config.getDouble("INPUT_kDriverTurnPCT", 0.85);     //1.0);
    public static final double kDriverStrafePCT = config.getDouble("INPUT_kDriverStrafePCT", 0.7);  //1.0);

    public static final double kTurretStickDeadBand = config.getDouble("INPUT_kTurretStickDeadBand", 0.80);
    public static final double kHoodStickDeadBand = config.getDouble("INPUT_kHoodStickDeadBand", 0.1);
    
  }

  public static final class Tower{

    public static final double kIntakePinchPower = config.getDouble("TOWER_Intake_Pinch_Power", .5);
    public static final double kWristMaxPower    = config.getDouble("TOWER_Wrist_Max_Power", .2);
    public static final double kElbowPCTPower    = config.getDouble("TOWER_Elbow_PCT_Power", 1.0);
    public static final double kShoulderPCTPower = config.getDouble("TOWER_Shoulder_PCT_Power", .85);
    public static final double kIntakeDeployMills = config.getDouble("INTAKE_Deploy_Mills", 1000);

    public static enum AutonFlags {IGNORE, ORIGIN, HIGHPLACE, MIDPLACE, HUMANPLAYERGRAB, GRABFROMINTAKE}
  }


  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = config.getDouble("AUTON_kMaxSpeedMetersPerSecond", 3.0);
    public static final double kMaxAccelerationMetersPerSecondSquared = config.getDouble("AUTON_kMaxAccelerationMetersPerSecondSquared", 3);
    public static final double kMaxAngularSpeedRadiansPerSecond = config.getDouble("AUTON_kMaxAngularSpeedRadiansPerSecond", Math.PI);          // nailed up, no config
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = config.getDouble("AUTON_kMaxAngularSpeedRadiansPerSecondSquared", Math.PI);   // nailed up, no config

    public static final double kPXController        = config.getDouble("AUTON_kPXController", 1);   // FWL not sure if they are used but if they are  we need to test
    public static final double kPYController        = config.getDouble("AUTON_kPYController", 1);
    public static final double kPThetaController    = config.getDouble("AUTON_kPThetaController", 1);

    public static int kTotalAutons                   = config.getInt("AUTON_TotalAutons", 5);
    
    /* public static double kAuton1_DriveForward        = config.getDouble("AUTON1_kDriveForward", 0.2);
    public static double kAuton1_DriveBackward       = config.getDouble("AUTON1_kDriveBackward", -0.5); //-0.2
    public static double kAuton1_DriveBackwardSlow   = config.getDouble("AUTON1_kDriveBackwardSlow", -0.1);
    public static double kAuton1_DriveBackTime       = config.getDouble("AUTON1_kDriveBackTime", 2.0);
    public static double kAuton1_DriveFromWallTime   = config.getDouble("AUTON1_kDriveFromWallTime", 4.0);
    public static double kAuton1_ShootingTime        = config.getDouble("AUTON1_kShootingTime", 6.0);

    public static double kAuton2_DriveForward        = config.getDouble("AUTON2_kDriveForward", 1.0);
    public static double kAuton2_DriveBackward       = config.getDouble("AUTON2_kDriveBackward", -0.5); //-0.2
    public static double kAuton2_DriveBackwardSlow   = config.getDouble("AUTON2_kDriveBackwardSlow", -0.1);
    public static double kAuton2_DriveBackTime       = config.getDouble("AUTON2_kDriveBackTime", 4.0);
    public static double kAuton2_DriveFromWallTime   = config.getDouble("AUTON2_kDriveFromWallTime", 4.0);
    public static double kAuton2_ShootingTime        = config.getDouble("AUTON2_kShootingTime", 6.0);
    public static double kAuton2_DriveStrafe         = config.getDouble("AUTON2_kDriveStrafe", -0.3);

    public static double kAuton3_DriveForward        = config.getDouble("AUTON3_kDriveForward", 1.0);
    public static double kAuton3_DriveBackward       = config.getDouble("AUTON3_kDriveBackward", -0.5); //-0.2
    public static double kAuton3_DriveBackwardSlow   = config.getDouble("AUTON3_kDriveBackwardSlow", -0.1);
    public static double kAuton3_DriveBackTime       = config.getDouble("AUTON3_kDriveBackTime", 4.0);
    public static double kAuton3_DriveFromWallTime   = config.getDouble("AUTON3_kDriveFromWallTime", 4.0);
    public static double kAuton3_ShootingTime        = config.getDouble("AUTON3_kShootingTime", 6.0);
    public static double kAuton3_DriveStrafe         = config.getDouble("AUTON3_kDriveStrafe", 2.0);
    */

    //public static double kReducedTimer = config.getDouble("AUTON_kTimer", 4.0);
    
    public static final double kTurretFullLeft   = config.getDouble("SHOOTR_kTurrretFullLeft", .318); //,0.35) ;
    public static final double kTurretFullRight = config.getDouble("SHOOTR_kTurretFullRight", .615); //0.620


    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static class MatchSettings{
    public static int      kAllianceColor         = 0;       // these will be set on robot class in init
    public static String   kAllianceColorName     = "unk";  // these will be set on robot class in init
    public static int      kBlueAlliance          = 1;  // these will be set on robot class in init
    public static int      kRedAlliance          = 2;  // these will be set on robot class in init

  }

  public static final class AutonStartSettings {  // these are specific to 2003 code
    public static int     kStartPosition_123     = config.getInt("AUTON_START_StartPosition", 2);
    public static int     kDropPosition_123      = config.getInt("AUTON_START_DropPosition", 2);
    public static boolean kScoreOnRamp           = config.getBoolean("AUTON_START_Score_On_Ramp", true);
  }


  public static final class AutonTestConstants {
    public static boolean kTestingGyroNavigate  = config.getBoolean("kTestingGyroNavigate",false); 
    public static double kGyroHeading1          = config.getDouble("AUTON_TEST_kGyroHeading1", 0.0);
    public static double kGyroHeading2          = config.getDouble("AUTON_TEST_kGyroHeading2", 0.0);
    public static double kGyroHeading3          = config.getDouble("AUTON_TEST_kGyroHeading3", 0.0);

  }

  public static final class TelemetrySwitches {
    public static boolean SwerveDisplayOn  = config.getBoolean("TELE_Swerve_On", true);
    public static boolean DriveDisplayOn   = config.getBoolean("TELE_Drive_On", true);
    public static boolean TowerDisplayOn   = config.getBoolean("TELE_Tower_On", true);
    public static boolean VisionDisplayOn  = config.getBoolean("TELE_Vision_On", false);
    public static boolean InputsDisplayOn  = config.getBoolean("TELE_Inputs_On", false);



  }

  public static final class ShooterConstants {
    public static final double kP_PidTuningValue = config.getDouble("SHOOTR_kPTuningValue", 0.22); 
    public static final double kI_PidTuningValue = config.getDouble("SHOOTR_kITuningValue", 0.0);
    public static final double kD_PidTuningValue = config.getDouble("SHOOTR_kDTuningValue", 0.0);
    public static final double kF_PidTuningValue = config.getDouble("SHOOTR_kFTuningValue", 0.047259);  // feed forward

    public static final double kP_TurretValue = config.getDouble("SHOOTR_kPTurretValue", 21.3); 
    public static final double kI_TurretValue = config.getDouble("SHOOTR_kPTurretValue", 0.0); 
    public static final double kD_TurretValue = config.getDouble("SHOOTR_kPTurretValue", 0.0); 

    public static final String aryAllianceColor[] = {"blue","red"};

    public static final int    kAllianceBlue = 0;
    public static final int    kAllianceRed = 1;

    public static final double kHoodPowerDeadBand   = config.getDouble("SHOOTR_kHoodPowerDeadBand",.05) ;
    public static final double kTurretPowerDeadBand = config.getDouble("SHOOTR_kTurretPowerDeadBand",.30) ;

    public static final double kHoodFullUp   = config.getDouble("SHOOTR_kHoodFullUp",.05) ;
    public static final double kHoodFullDown = config.getDouble("SHOOTR_kHoodFullDown",.05) ;

    public static final double kTurretFullLeft    = 0.36;//config.getDouble("SHOOTR_kTurrretFullLeft", .318); hardcoded at havoc
    public static final double kTurret45FromRight = config.getDouble("SHOOTR_kTurret45FromRight", .486); //0.580 //,0.65) ;
    public static final double kTurret90FromRight = config.getDouble("SHOOTR_kTurret90FromRight", .436); //0.580 //,0.65) ;
    public static final double kTurretForward     = config.getDouble("SHOOTR_kTurretForward", 0.536); 
    public static final double kTurretFullRight   = 0.54;//config.getDouble("SHOOTR_kTurretFullRight", 0.536); hardcoded at havoc
    public static final double kTurretExtendedRight = config.getDouble("SHOOTR_kTurretExtendedRight", 0.55); //was 0.55
    
    public static final double kHoodMaxPower = config.getDouble("SHOOTR_kHoodMaxPowerValue", 0.3);
    public static final double kPointBlankHood = 0.26;
    public static final double kTurretMaxPower = config.getDouble("SHOOTR_kTurretMaxPowerValue", 1.0 /*0.3*/);
    public static final int    kIntakeMotorPanelId = config.getInt("SHOOTR_kIntakeMotorPanelId", 12);  // distribtuion panel ID
    public static final double kTonyRollerPower = config.getDouble("SHOOTR_kTonyRollerPower", .75);

    //limelight height calculation
    //Measured by Zach on February 19, 2022
    public static final double kLimelightMountWidthMM = 37;
    public static final double kLimelightMountHeightMM = 41;

    public static final double kLimelightTiltAngleRadians = (40 * Math.PI)/180;
    public static final double kLimelightHeightMeters = 0.475 + 0.3;
    public static final double kTargetHeightMeters = 2.667;
    
    public static final double kDeltaHeightMeters = kTargetHeightMeters - kLimelightHeightMeters;


    // Proportional Response variabls for the turret  fwl - 02/26/2022
    public static double kTurret_PropRes_TargetCenter  = config.getDouble("TURRET_PropRes_TargetCenter", 47.00);
    public static double kTurret_PropRes_TargetWidth   = config.getDouble("TURRET_PropRes_TargetWidth",   0.50);
    public static double kTurret_PropRes_WindowWidth   = config.getDouble("TURRET_PropRes_WindowWidth",   8.00);
    public static double kTurret_PropRes_Proportion    = config.getDouble("TURRET_PropRes_WindowWidth",   0.10);



  }

  public static final class LimelightConstants {
    
    public static final double kXDeadBand    = config.getDouble("LIMEL_kXDeadBand",.05) ;
    public static final double kYDeadBand    = config.getDouble("LIMEL_kYDeadBand",.05) ;
    public static final double kAreaDeadBand = config.getDouble("LIMEL_kAreaDeadBand",.05) ;


  }

  public static final class ClimberConstants {
    
    public static int    kLift1RRBannerDigitalPort = config.getInt("CLIMB_kLift1RRBannerDigitalPort", 6); //0.4) ; // was 2 changed cuz problems
    // not final so these can change, yes we call the constants, get over it. 
    public static double kLiftNormalSpeed  =  config.getDouble("CLIMB_kLiftNormalSpeed", 1.0); //0.8) ;
    public static double kDartNormalSpeed  =  config.getDouble("CLIMB_kDartNormalSpeed", 1.0); //0.8) ;
    public static double kLift2InPower     =  kLiftNormalSpeed;  // makes this clearer for auton
    public static double kLift2OutPower    = -kLiftNormalSpeed;  


    // Very Important note: Modify this in case you change encoders. Change to keep your I/ Out positions correct. 
    // Set hand full out, adjust this and the porition read should be the full out posiiton on the screen. 
    public static double kLift2EncoderAdjust  = config.getDouble("CLIMB_kLift2EncoderAdjust",0.0);     
    public static double kLift2FullInPosit    = config.getDouble("CLIMB_kLift2FullInPosit", 0.93);    //0.93 
    public static double kLift2FullOutPosit   = config.getDouble("CLIMB_kLift2FullOutPosit",0.4249);  //0.4249

    // Very Important note: Modify this in case you change encoders. Change to keep your Up/Down positions correct. 
    // Set Dart at full allowed height. Then adjust this and the porition read should be the full Up posiiton on the screen. 
    public static double kDartEncoderAdjust  = config.getDouble("CLIMB_kDartEncoderAdjust",0.0);     
    public static double kDartFullUpPosit    = config.getDouble("CLIMB_kDartFullUpPosit",0.8725);     //0.9200 0.9665) ;     
    public static double kDartFullDownPosit   = config.getDouble("CLIMB_kDartFullDownPosit",0.1309);
    

    public static double kStep1SlowLiftRobotPower     = config.getDouble("CLIMB_kStep1SlowLiftRobotPower", 0.2); 
    public static double kStep1SlowLiftRobotPosit     = config.getDouble("CLIMB_kStep1SlowLiftRobotposit", 20.0); 
    public static double kStep1Lift2FullOutPosit      = kLift2FullOutPosit;   // end of arm
    public static double kStep1DartUpToBarPosit       = config.getDouble("CLIMB_kStep1DartUpToBarPosit", 0.834); ///*0.9150*/ 0.9665);
    

    public static double kStep2Lift2GrabPipePosit     = config.getDouble("CLIMB_kStep2Lift2GrabPipePosit", 0.45); // /*0.4500*/ 0.43) ;   // grab pipe
    public static double kStep2DartSwingUpRobotPosit  = config.getDouble("CLIMB_kStep2DartSwingUpRobotPosit", 0.6467);

    public static double kStep3Lift2PullRobotOffPosit = config.getDouble("CLIMB_kStep3Lift2PullRobotOffPosit", 0.5800);
    public static double kStep3DartSwingDownRobotPosit= config.getDouble("CLIMB_kStep3DartSwingDownRobotPosit", kDartFullUpPosit);
                                                        // arm full up make robot drop all the way down, may slow sway

    public static double kStep4DartSwingUpRobotPosit  = config.getDouble("CLIMB_kStep4DartSwingUpRobotPosit", 0.6467);

  }

  public static final class GyroNavigateConstants {

    public static double kPID_P = config.getDouble("GYRO_NAV_PID_P", 1.0);
    public static double kPID_I = config.getDouble("GYRO_NAV_PID_I", 0.0);
    public static double kPID_D = config.getDouble("GYRO_NAV_PID_D", 0.0);
    public static double kPowerProportion = config.getDouble("GYRO_NAV_Power_Proportion", 0.01);
    public static double kPowerDeadband = config.getDouble("GYRO_NAV_Power_Deadband", 0.04);
    public static double kMinPower = config.getDouble("GYRO_NAV_Min_Power", 0.075);

  }
  
}
