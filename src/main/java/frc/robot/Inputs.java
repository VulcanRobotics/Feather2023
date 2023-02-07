
package frc.robot;

import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShootSubsystem;

import com.fasterxml.jackson.databind.ser.std.StdArraySerializers.FloatArraySerializer;

import edu.wpi.first.util.EventVector;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Hi this is shaun

public class Inputs {
    private static XboxController m_driverXbox  = new XboxController(Constants.OIConstants.kDriverControllerPort);
    //private static Joystick m_driverControl   = new Joystick(Constants.OIConstants.kDriverControllerPort);

    private static Joystick m_operatorControl = new Joystick(Constants.OIConstants.kOperatorControllerPort);
    private static Joystick m_extraControl    = new Joystick(3); //OIConstants.kClimbButtonBoxPort);

        // Define all your variables here.

    public static boolean robotOperatingModeEndGame = false;        // not cleared out
    public static boolean robotOperatingModeNormal = true;         // not cleared out

    public static double driverPower = 0.0;
    public static double driverStrafe = 0.0;
    public static double driverTurn = 0.0;

    public static boolean driverTrigger = false;
    public static boolean leftDriverTrigger = false;
    public static boolean operatorTrigger = false;
    public static boolean operatorThumbSafety = false;

    public static boolean driverReverseControls = false;

    // anyone can do these but in this case all are operator
    public static double hoodAdjust = 0.0;
    public static double turretAdjust = 0.0;

    public static double xAxisJoystick = 0.0;
    public static double yAxisJoystick = 0.0;
    public static double zAxisJoystick = 0.0;

    public static double overrideTurretAngle = 0.0;
    public static boolean overrideTurret = false;
    
    public static double overrideFrontShooterMotorPower = 0.0;
    public static double overrideBackShooterMotorPower = 0.0;
    public static boolean overrideShooterMotors = false; 

    public static double shooterManualPower = 0.0;
    public static boolean turretTestAdjustLeft = false;
    public static boolean turretTestAdjustRight = false;
    public static double  turretDesiredPosit = 0.0;

    public static double overrideHoodAngle = 0.0;
    public static boolean overrideHood = false;
    

    public static boolean masterEndgameArm = false;                             // tell shooter subsystem that control are active for it
    public static boolean masterClimbAutoEnabled = false;                             // tell shooter subsystem that control are active for it
    public static boolean m1Override = false;                             // tell shooter subsystem that control are active for it
    public static boolean masterAutoEnabled    = false;                             // tell shooter subsystem that control are active for it

    public static boolean shooterFullAutoModeOn = false;                   // this will toggle a value in the shooter 
    public static boolean shooterFullAutoModeOff = false;                  // this will toggle a value in the shooter class
    public static boolean shooterTargettingAuto = false;
    public static boolean shooterTargettingManual = false;
    public static boolean shooterWheelAuto = false;
    public static boolean shooterWheelManual = false;
    public static boolean shooterReverseGibRoller = false;
    public static boolean shooterWeakShot = false;
    public static boolean shooterWallShot = false;


    public static double  climberLift1PowerOverride = 0.0;
    public static double  climberLift2PowerOverride = 0.0;
    public static double  climberDartPowerOverride = 0.0;

    public static double  climberRequestedPower = 0.0;
    public static boolean climberLift1Release = false;
    public static boolean climberLift1Up = false;
    public static boolean climberLift1Down = false;
    public static boolean climberLift2Up = false;
    public static boolean climberLift2Down =false;
    public static boolean climberDartUp = false;
    public static boolean climberDartDown = false;
    public static boolean climberDart110Angle = false;
    public static boolean climberDart50Angle = false;

    public static boolean driveResetGyro        = false;        // used in Drive subsystem to rest the gyro direction
    public static double  driveDesiredHeading   = 0.0;          // turn robot to a specific gyro heading
    public static boolean driveGoStraight       = false;
    public static boolean driveFieldRelitive    = false;
    public static boolean driveGyroCorrected    = false;

    public static double rotationInputOverride = 0.0;
    public static boolean rotationOverride = false;


    public static boolean intakePullIn      = false;
    public static boolean intakePushOut     = false;

    public static boolean intakeRetract     = false;
    public static boolean intakeDeploy      = false;

    public static String  recordingStatus   = "";

    public static int     testStepNumber = 0;
    public static boolean testStepIncrement = false;
    public static boolean testStepDecrement = false;
    public static boolean lastTestStepButtonPressed = false;

    // these must are not reset by zeroValues
    // these are changed by Inputs.setAuton while in display.periodic.
    public static int     autonToRun        = 0;
    public static int     autonToRunHighest = Constants.AutoConstants.kTotalAutons;
    public static double  autonDelay        = 0.0;
    public static boolean runAuto = false;                  // not clear out
 
    public static int     allianceColor     = Constants.ShooterConstants.kAllianceBlue;
 
    public static boolean fieldCentric = true;

    public static boolean inAutonMode = false;

    public static boolean driverIsUsingGamepad = true;

    public static boolean shootPhotogateSeesBall = false;

    public static boolean driverStickIsHeld = false;

    public static double initialYAW = 0.0;

    public static double drivePowerOffset = 0.0;
    public static double lastDPAD = -1.0;



    public static boolean hardMovementStop = false;
    public static boolean hardMovementBreak = false;

    public Inputs() {};             // Class constructor


    public static void periodic(){

        zeroValues(); // clear out all varible before we read again

        /////////////////////////////////////////////////////////////////
        // Extra Box control
        /////////////////////////////////////////////////////////////////
        masterEndgameArm        = m_extraControl.getRawButton(1);
        masterClimbAutoEnabled  = m_extraControl.getRawButton(2);
        masterAutoEnabled       = /*false; */m_extraControl.getRawButton(3);
        

        /////////////////////////////////////////////////////////////////
        // used by auto climb to adjust from normal power
        /////////////////////////////////////////////////////////////////
        climberLift1PowerOverride = 0.0;        
        climberLift2PowerOverride = 0.0;
        climberDartPowerOverride = 0.0;


        /////////////////////////////////////////////////////////////////
        // Driver controls  - showing all the steps to the new kids.
        /////////////////////////////////////////////////////////////////
        if (driverIsUsingGamepad) { 
            driverPower = applyDeadBand(-m_driverXbox.getLeftY(), Constants.DriveConstants.kJoystickDeadband); 
            driverPower *= (Constants.OIConstants.kDriverPowerPCT + drivePowerOffset); // reduce top power 
            //driverPower = getCubePower(driverPower);

            driverStrafe = applyDeadBand(-m_driverXbox.getLeftX(), Constants.DriveConstants.kJoystickDeadband); 
            driverStrafe *= (Constants.OIConstants.kDriverStrafePCT + drivePowerOffset);
            //driverStrafe = getCubePower(driverStrafe);

            //driverTurn = applyDeadBand( m_driverXbox.getRightX(), /*Constants.DriveConstants.kJoystickDeadband*/ 0.0); 
            driverTurn = applyDeadBand( m_driverXbox.getRightX(), Constants.DriveConstants.kJoystickDeadband); 
            driverTurn *= Constants.OIConstants.kDriverTurnPCT;
            //driverTurn = getCubePower(driverTurn);
            
        


            double DPAD = m_driverXbox.getPOV(0); //💀
            if (DPAD == 90 || DPAD == 270) {
                if (initialYAW < DriveSubsystem.m_gyro.getYaw() - 1.5) {
                    driverTurn = applyDeadBand(0.1, Constants.DriveConstants.kJoystickDeadband); 
                    driverTurn *= Constants.OIConstants.kDriverTurnPCT;
                } else if (initialYAW > DriveSubsystem.m_gyro.getYaw() + 1.5) {
                    driverTurn = applyDeadBand(-0.1, Constants.DriveConstants.kJoystickDeadband); 
                    driverTurn *= Constants.OIConstants.kDriverTurnPCT;
                }
            }

            if (m_driverXbox.getRightTriggerAxis() == 1){
                drivePowerOffset += 0.05;
            } else if (m_driverXbox.getLeftTriggerAxis() == 1) {
                drivePowerOffset -= 0.05;
            }



            if (masterAutoEnabled == true) {
                if (DriveSubsystem.m_gyro.getRoll() > 0) {
                    driverPower = applyDeadBand(-0.2, Constants.DriveConstants.kJoystickDeadband); 
                    driverPower *= (Constants.OIConstants.kDriverPowerPCT + drivePowerOffset);
                } else if (DriveSubsystem.m_gyro.getRoll() < 0) {
                    driverPower = applyDeadBand(0.2, Constants.DriveConstants.kJoystickDeadband); 
                    driverPower *= (Constants.OIConstants.kDriverPowerPCT + drivePowerOffset);
                }

            }


            /*if (DPAD != lastDPAD && DPAD != -1){
                if (DPAD == 0){
                    drivePowerOffset += 0.05;
                } else if (DPAD == 180){
                    drivePowerOffset -= 0.05;
                }
            } else*/ if (DPAD == 90) {
                driverStrafe = applyDeadBand(-(0.5), Constants.DriveConstants.kJoystickDeadband); 
                driverStrafe *= (Constants.OIConstants.kDriverStrafePCT + drivePowerOffset);
            } else if (DPAD == 270) {
                driverStrafe = applyDeadBand(-(-0.5), Constants.DriveConstants.kJoystickDeadband); 
                driverStrafe *= (Constants.OIConstants.kDriverStrafePCT + drivePowerOffset);
            } else {
                initialYAW = DriveSubsystem.m_gyro.getYaw();
            }

            if (DPAD == 0){
                driverPower = applyDeadBand(0.2, Constants.DriveConstants.kJoystickDeadband); 
                driverPower *= (Constants.OIConstants.kDriverPowerPCT + drivePowerOffset);
            } else if (DPAD == 180){
                driverPower = applyDeadBand(-0.2, Constants.DriveConstants.kJoystickDeadband); 
                driverPower *= (Constants.OIConstants.kDriverPowerPCT + drivePowerOffset);
            }

            lastDPAD = DPAD;

            drivePowerOffset = Math.max(Math.min(drivePowerOffset, 0.15), -0.85);

            int drivePowerInt = (int) ((0.85 + drivePowerOffset) * 100);
            String drivePowerDisplay = Integer.toString(drivePowerInt) + "% Power";

            SmartDashboard.putNumber("Drive Power Offset", drivePowerOffset);
            SmartDashboard.putString("Drive Power Maximum:", drivePowerDisplay);
            //SmartDashboard.putNumber("dpad angle", m_driverXbox.getPOV(0));


            driverTrigger = /*(m_driverXbox.getLeftTriggerAxis() > 0.5 ? true : false) ||*/ (m_driverXbox.getRightTriggerAxis() > 0.5 ? true : false);
            leftDriverTrigger = m_driverXbox.getLeftTriggerAxis() > 0.5 ? true : false;

            if (m_driverXbox.getBackButtonReleased()) { 
                fieldCentric = !fieldCentric;
            }

            driveResetGyro  = m_driverXbox.getStartButton();            // used in Drive subsystem to rest the gyro direction
            driveDesiredHeading = 0.0;                                  // turn robot to a specific gyro heading, changed in auton code
            driveGoStraight = false;                                    // tell system to go straight

            driverReverseControls = false;//m_driverXbox.getLeftTriggerAxis() > .5 ? true : false;
            if (driverReverseControls){
                driverPower  *= -1;
                driverStrafe  *= -1;
                driverTurn  *= -1;
              }
                  
            intakeDeploy =  m_driverXbox.getRightTriggerAxis() > .5 ? true : false;    
            //intakeRetract       = m_driverControl.getRawButton(4);
            //intakeDeploy        = m_driverControl.getRawButton(6);
            //intakePullIn        = m_driverControl.getRawButton(3);              // buttons on top of stick right side
            //intakePushOut       = m_driverControl.getRawButton(5);
    
    

        }

        Constants.telemetry.putNumber("INPUT Driver Power", driverPower, false);
        Constants.telemetry.putNumber("INPUT Driver Strafe", driverStrafe, false);
        Constants.telemetry.putNumber("INPUT Driver Turn", driverTurn, false);

        //SmartDashboard.putNumber("Inputs.driverPower", driverPower);

        // commented out so that the system will not try to service a joystick that 
        // is not there. 
        /* else { 
            driverPower = applyDeadBand(-m_driverControl.getY(), Constants.DriveConstants.kJoystickDeadband); 
            driverPower *= Constants.OIConstants.kDriverPowerPCT; // reduce top power 
            //driverPower = getCubePower(driverPower);

            driverStrafe = applyDeadBand(-m_driverControl.getX(), Constants.DriveConstants.kJoystickDeadband); 
            driverStrafe *= Constants.OIConstants.kDriverStrafePCT;
            //driverStrafe = getCubePower(driverStrafe);

            driverTurn = applyDeadBand( m_driverControl.getZ(), Constants.DriveConstants.kJoystickDeadband); 
            driverTurn *= Constants.OIConstants.kDriverTurnPCT;
            //driverTurn = getCubePower(driverTurn);

            driverTrigger = m_driverControl.getTrigger();
            
            if (m_driverControl.getRawButtonReleased(3)) { 
                fieldCentric = !fieldCentric;
            }

            driveResetGyro  = m_driverControl.getRawButton(2);          // used in Drive subsystem to rest the gyro direction
            driveDesiredHeading = 0.0;                                  // turn robot to a specific gyro heading, changed in auton code
            driveGoStraight = false;                                    // tell system to go straight

            if( m_driverControl.getRawButton(12) ) {
                recordingStatus = "Recording";
            }else if( m_driverControl.getRawButton(11) ){
                recordingStatus = "";
            }

            //intakeRetract       = m_driverControl.getRawButton(4);
            intakeDeploy        = m_driverControl.getRawButton(6);
            //intakePullIn        = m_driverControl.getRawButton(3);              // buttons on top of stick right side
            //intakePushOut       = m_driverControl.getRawButton(5);


        }*/


        if( masterEndgameArm ){                 // are we in the endgame? 
            robotOperatingModeEndGame = true;
            robotOperatingModeNormal = false;
        } else {
            robotOperatingModeEndGame = false;
            robotOperatingModeNormal = true;
        }   

        Constants.telemetry.putTrueBoolean("Robot Mode End Game", robotOperatingModeEndGame, true); // saved and forced display
        Constants.telemetry.putTrueBoolean("Robot Mode Normal", robotOperatingModeNormal, true);    // saved and forced display
        Constants.telemetry.putTrueBoolean("INPUT Master Endgame Arm", masterEndgameArm, false); // saved and forced display


        /////////////////////////////////////////////////////////////////
        // Operator controls
        /////////////////////////////////////////////////////////////////


        operatorTrigger    = m_operatorControl.getTrigger();
        operatorThumbSafety    = m_operatorControl.getRawButton(2);  // thumb button on stick side

        turretAdjust    = applyDeadBand(m_operatorControl.getZ(), Constants.OIConstants.kTurretStickDeadBand); 
        turretAdjust    *= Constants.ShooterConstants.kTurretMaxPower;

        hoodAdjust      = applyDeadBand(m_operatorControl.getY(), Constants.OIConstants.kHoodStickDeadBand); 
        hoodAdjust      *= Constants.ShooterConstants.kHoodMaxPower;

        xAxisJoystick = m_operatorControl.getX();
        yAxisJoystick = m_operatorControl.getY();
        zAxisJoystick = m_operatorControl.getZ();

        shooterManualPower = convertToLinearPower(m_operatorControl.getThrottle()); // what does CBU want

        

        shooterFullAutoModeOff = (m_operatorControl.getRawButton(7) || m_operatorControl.getRawButton(9) || m_operatorControl.getRawButton(11));  // IMHO two butttons are safer 
        shooterFullAutoModeOn = (m_operatorControl.getRawButton(8) || m_operatorControl.getRawButton(10) || m_operatorControl.getRawButton(12)); 

        shooterReverseGibRoller = m_operatorControl.getRawButton(3);
        shooterWeakShot = m_operatorControl.getRawButton(4);

        
        if( m_operatorControl.getRawButton(6) || m_extraControl.getRawButton(4)){ // if we have the top right button of the operator joystick or switch 4 pressed
            shooterWallShot = true; //shortshot is initiated and manual overide is active
        }
       
        if( !masterEndgameArm ){
            
            recordingStatus = "";
            //runAuto  = m_extraControl.getRawButton(2);  // run the currently selected auton

            if(m_extraControl.getRawButton(3)){          // switch on extra control
                recordingStatus = "record";
            }

            /*
            if( Constants.AutonTestConstants.kTestingGyroNavigate == true){ // set in config file
                //used to test the gyro drive stuff uncomment for testing

                if(m_extraControl.getRawButton(3)){              // switch on extra control
                    driveGyroCorrected = true;
                    fieldCentric = false;
                    recordingStatus = "record";

                    if(m_extraControl.getRawButton(5))                // lift release button, set to 0 heading
                        driveDesiredHeading = 0.0;
                    else if(m_extraControl.getRawButton(8))           // 1st toggle button top
                        driveDesiredHeading = -Constants.AutonTestConstants.kGyroHeading1;
                    else if(m_extraControl.getRawButton(7))           // 1st toggle button down
                        driveDesiredHeading = Constants.AutonTestConstants.kGyroHeading1;                  
                    else if(m_extraControl.getRawButton(10))          // 2nd toggle button top
                        driveDesiredHeading = -Constants.AutonTestConstants.kGyroHeading2;
                    else if(m_extraControl.getRawButton(9))           // 2nd toggle button down
                        driveDesiredHeading = Constants.AutonTestConstants.kGyroHeading2;
                    else if(m_extraControl.getRawButton(12))          // 3rd toggle button top
                        driveDesiredHeading = -Constants.AutonTestConstants.kGyroHeading3;
                    else if(m_extraControl.getRawButton(11))          // 3rd toggle button down
                        driveDesiredHeading = Constants.AutonTestConstants.kGyroHeading3;

                }
            }*/

        } else if( masterEndgameArm ){
            masterClimbAutoEnabled  = m_extraControl.getRawButton(2);

            //turretTestAdjustLeft = m_operatorControl.getRawButton(7);
            //turretTestAdjustRight = m_operatorControl.getRawButton(8);

            climberLift1Release = m_extraControl.getRawButton(5);
            climberLift1Up      = m_extraControl.getRawButton(8);
            climberLift1Down    = m_extraControl.getRawButton(7); 
            climberDartUp       = m_extraControl.getRawButton(10);
            climberDartDown     = m_extraControl.getRawButton(9);  
            climberLift2Up      = m_extraControl.getRawButton(12);
            climberLift2Down    = m_extraControl.getRawButton(11);
            m1Override = m_extraControl.getRawButton(4);
        }

        hardMovementStop = m_driverXbox.getAButton() && m_driverXbox.getBButton();
        Constants.telemetry.putTrueBoolean("HARD MOVEMENT STOP", hardMovementStop, true);

        hardMovementBreak = m_driverXbox.getYButton();
        Constants.telemetry.putTrueBoolean("HARD MOVEMENT BREAK", hardMovementBreak, true);
    }  



    // This is used in autonomous to zero out all input variables
    // May not be needed as long as no one touches the remotes during autonomous.
    public static void zeroValues(){

        // Beginning of DO NOT RESET Variables
        // Below list all INPUT variables that are not supposed to be rest on each pass. 
        // They are included here to let all programmers know that they
        // MUST NOT BE SET TO A DEFAULT VALUE here. 

            // fieldCentric                 // Do not zero this out as this is carried over between each pass. 
            // climberLift1Lock             // Do not reset to default.
            // recordingStatus              // Do not erase this
            // driveDesiredHeading          // turn robot to a specific gyro heading


        // End of DO NOT RESET Variables


        // Beginning of the RESETTABLE Varibles. 
        // All values below here will be reset each pass to a default on each pass.
        // then we will read he CBU inputs to set them accordingly for that pass. 
        shooterWeakShot = false;
        shooterWallShot = false;

        shootPhotogateSeesBall = false;
        

        masterEndgameArm = false;            // extra box 1                     // extra box controls used in several places
        masterClimbAutoEnabled = false;      // extra box 2                     // tell shooter subsystem that control are active for it
        masterAutoEnabled    = false;        // extra box 3                     // tell shooter subsystem that control are active for it
        m1Override  = false;        // extra box 4                     // tell shooter subsystem that control are active for it
        runAuto              = false;
    
        driveResetGyro     = false;          // used in Drive subsystem to rest the gyro direction
        driveGoStraight    = false;          // tell system to go straight
        driveGyroCorrected = false;          // shut off gyro coirrection, will be turned on later if needed

        driverPower     = 0.0;
        driverStrafe    = 0.0;
        driverTurn      = 0.0;

        driverTrigger   = false;
        leftDriverTrigger = false;

        operatorTrigger = false;
        turretAdjust    = 0.0; 
        turretDesiredPosit = 0.0;
        hoodAdjust      = 0.0;
        intakeDeploy    = false;
        intakeRetract   = false;
        turretTestAdjustLeft = false;
        turretTestAdjustRight = false;


        shooterManualPower = 0.0; // what does CBU want

        intakePullIn = false;              // buttons on top of stick right side
        intakePushOut = false;
    
        shooterFullAutoModeOff = false;
        shooterFullAutoModeOn = false;

        shooterTargettingManual = false;
        shooterTargettingAuto = false;

        shooterWheelManual = false;
        shooterWheelAuto   = false;

        climberRequestedPower = 0.0;
        climberDart110Angle = false;
        climberDart50Angle = false;

        climberLift1Up = false;
        climberLift1Down = false;
        climberLift2Up = false;
        climberLift2Down = false;
        climberDartUp = false;
        climberDartDown = false;

        hardMovementStop = false;
        hardMovementBreak = false;
    }

    public static void setAuton(){

        // change the auton number
        //if( m_extraControl.getRawButtonReleased(1) == false )  // master arm button, red switch on extra control
        //    return;

        if( m_extraControl.getRawButtonReleased(8) == true){  ///Top button
            autonToRun++;
        } else if( m_extraControl.getRawButtonReleased(7) == true){  ///Bottom button
            autonToRun--;
        }
        // force us to within limits
        autonToRun = autonToRun < 0 ? 0 : autonToRun ; 
        autonToRun = autonToRun > Constants.AutoConstants.kTotalAutons ? 
                                Constants.AutoConstants.kTotalAutons : autonToRun ; 

        // now do auton delay
        if( m_extraControl.getRawButtonReleased(10) == true){  ///Top button
            autonDelay += .25;
        } else if( m_extraControl.getRawButtonReleased(9) == true){  ///Bottom button
            autonDelay -= .25;
        }
        // force us to within limits
        autonDelay = autonDelay < 0.0 ? 0.0 : autonDelay ; 
        autonDelay = autonDelay > 10.0 ? 10.0 : autonDelay ;

        SmartDashboard.putString("INPUT AUTON To Run", String.valueOf(autonToRun));
        SmartDashboard.putString("INPUT AUTON Delay", String.valueOf(autonDelay));
        SmartDashboard.putString("ROBOT ALIANCE Color", 
                                            DriverStation.getAlliance().toString().toLowerCase()); 

    }


    public static void saveTelem(){     // this is called in robot.periodic so we can pick up any auton changes.

        Constants.telemetry.saveString("*RECORD", recordingStatus );
        
        Constants.telemetry.saveTrueBoolean("INPUT Lift1 Button Up", Inputs.climberLift1Up );
        Constants.telemetry.saveTrueBoolean("INPUT Lift1 Button Down", Inputs.climberLift1Down );
        Constants.telemetry.saveTrueBoolean("INPUT Lift2 Button Up", Inputs.climberLift2Up );
        Constants.telemetry.saveTrueBoolean("INPUT Lift2 Button Down", Inputs.climberLift2Down );
        Constants.telemetry.saveTrueBoolean("INPUT Dart Button Up", Inputs.climberDartUp );
        Constants.telemetry.saveTrueBoolean("INPUT Dart Button Down", Inputs.climberDartDown );

        Constants.telemetry.putNumber("INPUT Driver Power", driverPower, false);
        Constants.telemetry.putNumber("INPUT Driver Strafe", driverStrafe, false);
        Constants.telemetry.putNumber("INPUT Driver Turn", driverTurn, false);

        Constants.telemetry.putTrueBoolean("INPUT Driver Trigger", driverTrigger, false);
        //Constants.telemetry.putTrueBoolean("INPUT Driver Field Centric", fieldCentric, true);
        Constants.telemetry.putTrueBoolean("INPUT Driver Reset Gyro", driveResetGyro);

        Constants.telemetry.putTrueBoolean("INPUT Shooter Auto On", shooterFullAutoModeOn);
        Constants.telemetry.putTrueBoolean("INPUT Shooter Auto Off", shooterFullAutoModeOff);

        Constants.telemetry.putString("INPUT AUTON To Run", String.valueOf(autonToRun),true);
        Constants.telemetry.putString("INPUT AUTON Delay", String.valueOf(autonDelay),true);
        Constants.telemetry.putString("INPUT ALIANCE Color", 
                                        Constants.ShooterConstants.aryAllianceColor[allianceColor],false);

        
        Constants.telemetry.putTrueBoolean("INPUT Operator Trigger", operatorTrigger, false);
        Constants.telemetry.putTrueBoolean("INPUT Operator Thumb Safety", operatorThumbSafety, false);
        Constants.telemetry.putNumber("INPUT Turret Manual Adjust", turretAdjust, false);

        Constants.telemetry.putNumber("INPUT Turret Override Angle", overrideTurretAngle, false);
        Constants.telemetry.putBoolean("INPUT Turret Override", overrideTurret, false);
        


        //Constants.telemetry.putTrueBoolean("INPUT Intake Retract", intakeRetract, false);
        Constants.telemetry.putTrueBoolean("INPUT Intake Deploy", intakeDeploy, false);
        Constants.telemetry.putTrueBoolean("INPUT Intake Pull In", intakePullIn, false);
        Constants.telemetry.putTrueBoolean("INPUT Intake Push Out", intakePushOut, false);
        Constants.telemetry.putNumber("INPUT Hood Manual Adjust", hoodAdjust, false);

    }
    /* This function is used to keep a value from going over  alimit. Kinda like a MAX

    public double getOverrideValue(double dInputValue, double dInputValueOverride, double dLimit){

        if( Math.abs(dInputValueOverride) > dLimit)   // operator o
            return dInputValueOverride;
        else
            return dInputValue;

    }




    /* Thus function will convert a joystick pct power -1.0 to 1.0 to a linear scale from 0.0 to 1.0
     * This is typically used for the Jotstick.Throttle() or the XBox Gamepad. Left and right Triggers
     *      Truth Table
     * Input| Step 1 | Step 2 | 
     *  1.0 |  0.00  |  0.00  | 
     *  0.5 |  0.50  |  0.25  |
     *  0.0 |  1.00  |  0.50  |
     * -0.5 |  1.50  |  0.75  |
     * -1.0 |  2.00  |  1.00  |
     * 
     * See steps below
    */

    public static double convertToLinearPower(double pctPower) {

        double step1Result = 1.0 - pctPower;     // step 1  - subtract from 1.0 to shift 1.0 to the left, results are 0.0 to 2.0. 
        double step2Result = step1Result / 2.0;  // step 2  - now divide by 2.0 to bring range to 0.0 to 1.0
        return step2Result;

    }


    public static int applyDeadBand( int value, int minValue ) {

        if( Math.abs( value ) < minValue){
            value = 0;
        }

        return value;
    }

    public static double applyDeadBand( double value, double minValue ) {

        if( Math.abs( value ) < minValue){
            value = 0.0;
        }

        return value;
    }

    public static double getSquarePower(double dInPower){
        return dInPower * Math.abs( dInPower ) ;       
    }

    public static double getCubePower(double dInPower){
        return dInPower * Math.abs(dInPower * dInPower);   // yes this is not needed to perserve the sign, just go with it. 
    }

    public static double getQuadPower(double dInPower){
        return dInPower * Math.abs(dInPower * dInPower * dInPower);
    }

}


