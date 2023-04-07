
package frc.robot;

import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.Tower.AutonIntakeFlags;
import frc.robot.Constants.Tower.AutonTowerFlags;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
//import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import javax.swing.text.StyledEditorKit.BoldAction;

import com.fasterxml.jackson.annotation.JsonSubTypes.Type;
import com.fasterxml.jackson.databind.ser.std.StdArraySerializers.FloatArraySerializer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.EventVector;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Hi this is shaun

public class Inputs {
    public static XboxController m_driverXbox  = new XboxController(Constants.OIConstants.kDriverControllerPort);
    public static Joystick m_operatorControl = new Joystick(Constants.OIConstants.kOperatorControllerPort);
    public static Joystick m_extraControl    = new Joystick(2); //OIConstants.kClimbButtonBoxPort);
    public static VisionSubsystem m_visionSubsystem = new VisionSubsystem();

        // Define all your variables here.

    // set by the driver inputs section
    public static double driverPower = 0.0;
    public static double driverStrafe = 0.0;
    public static double driverTurn = 0.0;
    public static double turboPower = 1;

    public static boolean driverReverseControls = false;

    // Operator controls
    //public static double xAxisJoystick = 0.0;
    //public static double yAxisJoystick = 0.0;
    //public static double zAxisJoystick = 0.0;

    // these are set by the operator set by FWL to make is easier to read. 
    public static double  towerShoulderPower = 0.0;
    public static double  towerElbowPower   = 0.0;
    public static double  towerWristSpeed    = 0.0;
    public static boolean towerClawOpenState  = false;

    public static boolean armReachedTarget = false;

    public static double targetWristPosition = 0.0;

    public static Constants.Tower.AutonTowerFlags autonRequestTowerGoTo = AutonTowerFlags.IGNORE;

    public static Constants.Tower.AutonIntakeFlags autonRequestIntakeGoTo = AutonIntakeFlags.IGNORE;
    public static double rightPincerMotorSpeed = 0.0;
    public static double leftPincerMotorSpeed = 0.0;

    //public static boolean m1Override = false;  

    public static boolean driveResetGyro        = false;    
    public static double  driveDesiredHeading   = 0.0;      
    public static boolean driveGoStraight       = false;
    public static boolean driveFieldRelitive    = false;
    public static boolean driveGyroCorrected    = false;

    public static double rotationInputOverride = 0.0;
    public static boolean rotationOverride = false;
    public static boolean swerveEncoderReset = false;
    public static boolean driveSwerveEncoderReset = false;
    public static boolean driveZeroGyro           = false;
    public static boolean driveToggleBrakeMode    = false;
    public static boolean driveToggleAutoPoseMode = false;
  
    
    public static boolean yellowLED = false;
    public static boolean purpleLED = false;
    public static boolean redLED = false;
    public static boolean greenLED = false;

    // these are set by the driver 
    public static boolean intakeDeploy   = false;
    public static boolean intakePinch     = false;
    public static boolean intakePinchIn   = false;
    public static boolean intakePinchOut  = false;

    public static int     testStepNumber        = 0;
    public static boolean testStepIncrement     = false;
    public static boolean testStepDecrement     = false;
    public static boolean lastTestStepButtonPressed = false;

    // these must are not reset by zeroValues
    // these are changed by Inputs.setAuton while in display.periodic.
    public static int     autonToRun        = 1;
    public static int     autonToRunHighest = Constants.AutoConstants.kTotalAutons;
    public static double  autonDelay        = 0.0;
    public static String  autonMessage = "No Auton Selected";

    public static boolean runAuto = false;                  // not clear out
 
    public static int     allianceColor     = Constants.ShooterConstants.kAllianceBlue;
 
    public static boolean fieldCentric = true;

    public static boolean inAutonMode = false;

    public static boolean endGame = false;

    // power massaging experiment. 
    public static double initialYAW = 0.0;
    public static double currentYAW = 0.0;
    public static double drivePowerOffset = 0.0;
    public static double lastDPAD = -1.0;

    public static boolean startedBalance = false;

    public Inputs() {};             // Class constructor

    public static void autoCenter(int pipeline_number){
        /* 
        if (Math.abs(DriveSubsystem.m_gyro.getYaw()) > 1.0){
            driverTurn = m_visionSubsystem.visionAdjustX()[1];
        } else if (!m_visionSubsystem.areWeCentered()){
            driverStrafe = m_visionSubsystem.visionAdjustX()[0];
        }*/

        if (((m_visionSubsystem.pipelineNumber != pipeline_number)) && pipeline_number != -1) {
            m_visionSubsystem.switchPipeline(pipeline_number);
        }

        driverStrafe = m_visionSubsystem.visionAdjustX(false)[0];

    }

    public static boolean seekTarget() {
        if (IntakeSubsystem.getHaveCube()) {
            driverTurn = 0.0;
            return true;
        } else {
            driverTurn = -m_visionSubsystem.visionAdjustX(true)[1];
            return false;
        }
    }

    public static void periodic(){

        zeroValues(); // clear out all varible before we read again
        setAuton();
         boolean bForceDisplay = Constants.DashboardSwitches.InputsDisplayOn;
        /////////////////////////////////////////////////////////////////
        // Extra Box control
        /////////////////////////////////////////////////////////////////
        //masterEndgameArm        = m_extraControl.getRawButton(1);
        //masterClimbAutoEnabled  = m_extraControl.getRawButton(2);
        

        /////////////////////////////////////////////////////////////////
        // Driver controls  - showing all the steps to the new kids.
        //                    instead of 1 big un readable statement. 
        /////////////////////////////////////////////////////////////////
        driverPower = applyDeadBand(m_driverXbox.getLeftY(), Constants.DriveConstants.kJoystickDeadband); 
        driverPower *= m_driverXbox.getRightBumper() ? turboPower : (Constants.OIConstants.kDriverPowerPCT + drivePowerOffset); // reduce top power 
        driverPower = getCubePower(driverPower);

        driverStrafe = applyDeadBand(m_driverXbox.getLeftX(), Constants.DriveConstants.kJoystickDeadband); 
        driverStrafe *= m_driverXbox.getRightBumper() ? turboPower : (Constants.OIConstants.kDriverStrafePCT + drivePowerOffset);
        driverStrafe = getCubePower(driverStrafe);

        driverTurn = applyDeadBand( m_driverXbox.getRightX(), Constants.DriveConstants.kJoystickDeadband); 
        driverTurn *= m_driverXbox.getRightBumper() ? turboPower :(Constants.OIConstants.kDriverTurnPCT  + drivePowerOffset); //Shaun added "+driverpoweroffset 3/22 post SCH"
        //driverTurn = getCubePower(driverTurn);

        // Process below is designed to override the different powers and allow us to set upper limiys
        // It is experimental. 
        double DPAD = m_driverXbox.getPOV(0); 
        if (DPAD == 90 || DPAD == 270) {
            if (0 < DriveSubsystem.m_gyro.getYaw() - 1.5) {
                driverTurn = applyDeadBand(-0.1, Constants.DriveConstants.kJoystickDeadband); 
                driverTurn *= Constants.OIConstants.kDriverTurnPCT;
            } else if (0 > DriveSubsystem.m_gyro.getYaw() + 1.5) {
                driverTurn = applyDeadBand(0.1, Constants.DriveConstants.kJoystickDeadband); 
                driverTurn *= Constants.OIConstants.kDriverTurnPCT;
            }
        }

        if (DPAD == 90) {
            driverStrafe = applyDeadBand(0.5, Constants.DriveConstants.kJoystickDeadband); 
            driverStrafe *= (Constants.OIConstants.kDriverStrafePCT + drivePowerOffset);
        } else if (DPAD == 270) {
            driverStrafe = applyDeadBand(-0.5, Constants.DriveConstants.kJoystickDeadband); 
            driverStrafe *= (Constants.OIConstants.kDriverStrafePCT + drivePowerOffset);
        } else {
            initialYAW = DriveSubsystem.m_gyro.getYaw();
        }
        
        /*if (DriveSubsystem.m_gyro.getYaw() < 0.0) {
            currentYAW = DriveSubsystem.m_gyro.getYaw() + 360;
        } else {
            currentYAW = DriveSubsystem.m_gyro.getYaw();
        }*/
        currentYAW = DriveSubsystem.m_gyro.getYaw();

        if (DPAD != lastDPAD && DPAD != -1){
            if (DPAD == 0){
                drivePowerOffset += 0.05;
            } else if (DPAD == 180){
                drivePowerOffset -= 0.05;
            }
        }/*else if (DPAD == 0){
            drivePowerOffset += 0.01;
        } else if (DPAD == 180){
            drivePowerOffset -= 0.01;
        } */

        lastDPAD = DPAD;

        //drivePowerOffset = Math.max(Math.min(drivePowerOffset, 0.15), -0.85);
        if (drivePowerOffset < -0.45){
            drivePowerOffset = -0.45;
        } else if (drivePowerOffset > 0.2){
            drivePowerOffset = 0.2;
        }

    
        
        //int drivePowerInt = (int) ((0.85 + drivePowerOffset) * 100);
        //String drivePowerDisplay = Integer.toString(drivePowerInt) + "% Power";

        // telemerty here will seve to telemetry record and the trys pushed to the dashboard. 

        /*if (m_driverXbox.getBackButtonReleased()) { 
            fieldCentric = !fieldCentric;               // flip the vale to true or false. 
        }*/

        driveResetGyro  = m_driverXbox.getStartButton();            // used in Drive subsystem to rest the gyro direction
        driveDesiredHeading = 0.0;                                  // turn robot to a specific gyro heading, changed in auton code
        driveGoStraight = false;                                    // tell system to go straight

        driverReverseControls = false;//m_driverXbox.getLeftTriggerAxis() > .5 ? true : false;
        if (driverReverseControls){
            driverPower  *= -1;
            driverStrafe  *= -1;
            driverTurn  *= -1;
        }
                
        intakeDeploy    =  m_driverXbox.getRightTriggerAxis() > .5 ? true : false;    
        intakePinch      =  m_driverXbox.getLeftTriggerAxis() > .5 ? true : false; 
        intakePinchIn    =  m_driverXbox.getRightBumperPressed() ? true : false;    
        intakePinchOut   =  m_driverXbox.getLeftBumperPressed() ? true : false;    

        driveSwerveEncoderReset = m_driverXbox.getStartButtonReleased();
        driveZeroGyro           = m_driverXbox.getAButtonReleased();
        driveToggleBrakeMode    = m_driverXbox.getBButtonReleased();
        //driveToggleAutoPoseMode = m_driverXbox.getYButtonReleased();
      


        /////////////////////////////////////////////////////////////////
        // Operator controls
        /////////////////////////////////////////////////////////////////

        towerElbowPower    = m_operatorControl.getX();
        towerShoulderPower = m_operatorControl.getY();
        towerWristSpeed    = m_operatorControl.getZ();

        endGame = m_extraControl.getRawButton(1);

        if(m_driverXbox.getYButtonPressed()){
            double currentPipeline = m_visionSubsystem.pipelineNumber;
            if (currentPipeline == 0) {
                m_visionSubsystem.switchPipeline(1);
            } else {
                m_visionSubsystem.switchPipeline(0);
            }
        }
        
        if (m_driverXbox.getLeftBumper()) {
            seekTarget();   
        }

        if (endGame){
            if ((Math.abs(DriveSubsystem.m_gyro.getRoll()) < 2.5 && startedBalance)){//&& Math.abs(DriveSubsystem.m_gyro.getYaw()) > 160) ||
            //(Math.abs(DriveSubsystem.m_gyro.getPitch()) < 2.5 && Math.abs(DriveSubsystem.m_gyro.getYaw()) > 70 && Math.abs(DriveSubsystem.m_gyro.getYaw()) < 110) && startedBalance == true){
                greenLED = true;
            } else{
                if (Math.abs(DriveSubsystem.m_gyro.getRoll()) > 10 || Math.abs(DriveSubsystem.m_gyro.getPitch()) > 10){
                    startedBalance = true;
                }
                redLED = true;
            }
        } else{
            if(m_extraControl.getRawButton(3)){ // if we have the top right button of the operator joystick or switch 4 pressed
                yellowLED = true;
            } else {
                yellowLED = false;
            }
            if(m_extraControl.getRawButton(4)){ // if we have the top right button of the operator joystick or switch 4 pressed
            purpleLED = true; 
            } else {
                purpleLED = false;
            }
        }
        


        SmartDashboard.putNumber("driverPo", drivePowerOffset * 100);
        SmartDashboard.putNumber("driver strafe", driverStrafe);
        SmartDashboard.putNumber("drivePowerSPEED", (Constants.OIConstants.kDriverStrafePCT+drivePowerOffset)*100);
        SmartDashboard.putNumber("Driver ROT", driverTurn);
        SmartDashboard.putNumber("currentYAW", currentYAW);


        //Alignment based on vision
        if (m_driverXbox.getXButton()){
            
            //Still jitters a little when calling this (a bit less though) - how can that be fixed?
            if (m_visionSubsystem.areWeCentered(3)) {
                driverPower = 0.2;
            } /*else {
                autoCenter();
            }*/
            if (!m_visionSubsystem.areWeCentered(2)) {
                autoCenter(-1);
            }

        }
        
        // saveTelem(); do not do this here. We will do it after auton in robot teleop and auton.

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
        runAuto              = false;
    
        driveResetGyro     = false;          // used in Drive subsystem to rest the gyro direction
        driveGoStraight    = false;          // tell system to go straight
        driveGyroCorrected = false;          // shut off gyro coirrection, will be turned on later if needed

        driverPower     = 0.0;
        driverStrafe    = 0.0;
        driverTurn      = 0.0;

        intakeDeploy    = false;
        intakePinch      = false;
        intakePinchIn    = false;
        intakePinchOut   = false;

        autonRequestTowerGoTo = AutonTowerFlags.IGNORE;
       

        rightPincerMotorSpeed = 0.0;
        leftPincerMotorSpeed = 0.0;

        targetWristPosition = 0.0;

    }

    
    public static void setAuton(){

        // change the auton number
        //if( m_extraControl.getRawButtonReleased(1) == false )  // master arm button, red switch on extra control
        //    return;

        if( m_operatorControl.getRawButtonReleased(12) == true){  ///Top button
            autonToRun++;
        } else if( m_operatorControl.getRawButtonReleased(11) == true){  ///Bottom button
            autonToRun--;
        }
        // force us to within limits
        autonToRun = autonToRun < 0 ? 0 : autonToRun ; 
        autonToRun = autonToRun > Constants.AutoConstants.kTotalAutons ? 
                                Constants.AutoConstants.kTotalAutons : autonToRun ; 

        // now do auton delay
        /* 
        if( m_operatorControl.getRawButtonReleased(10) == true){  ///Top button
            autonDelay += .25;
        } else if(m_operatorControl.getRawButtonReleased(9) == true){  ///Bottom button
            autonDelay -= .25;
        }
        // force us to within limits
        autonDelay = autonDelay < 0.0 ? 0.0 : autonDelay ; 
        autonDelay = autonDelay > 10.0 ? 10.0 : autonDelay ;
*/      if (autonToRun == 0) {
            autonMessage = "0: No auton currently";
        } else if (autonToRun == 1) {
            autonMessage = "1: Drop cone high, balance (18p)";
        } else if (autonToRun == 2) {
            autonMessage = "2: Drop cone high, leave community, balance (21p)";
        } else if (autonToRun == 3) {
            autonMessage = "3: Drop cone high, drive out of community (9p)";
        } else if (autonToRun == 4) {
            autonMessage = "4 RIGHT SIDE: Drop cone high, drive out and grab cube, come back and drop high (15p)";
        } else if (autonToRun == 5) {
            autonMessage = "4 LEFT SIDE: Drop cone high, drive out and grab cube, come back and drop high (15p)";
        }
        SmartDashboard.putNumber("INPUT AUTON To Run", autonToRun);
        SmartDashboard.putString("Running What Auton?", autonMessage);  
        //SmartDashboard.putString("INPUT AUTON Delay", String.valueOf(autonDelay));
        //SmartDashboard.putString("ROBOT ALIANCE Color", 
        //                                    DriverStation.getAlliance().toString().toLowerCase()); 

    }


    public static void saveTelem(){     // this is called in robot.periodic so we can pick up any auton changes.
    // YK - Disabled telemetry 2023-02-27
    /*
        Constants.telemetry.putNumber("INPUT Driver Power", driverPower, true);
        Constants.telemetry.putNumber("INPUT Driver Strafe", driverStrafe, true);
        Constants.telemetry.putNumber("INPUT Driver Turn", driverTurn, true);

        Constants.telemetry.putString("INPUT Auton Goto", autonRequestTowerGoTo.name(), true);
        Constants.telemetry.putNumber("Drive Power Offset", drivePowerOffset, true);
        if( Constants.TelemetrySwitches.InputsDisplayOn){
            

            Constants.telemetry.putTrueBoolean("INPUT Driver Field Centric", fieldCentric, true);
            Constants.telemetry.putTrueBoolean("INPUT Driver Reset Gyro", driveResetGyro, true);

            Constants.telemetry.putTrueBoolean("INPUT Intake Deploy", intakeDeploy);
            Constants.telemetry.putTrueBoolean("INPUT Intake Pinch", intakePinch);
            Constants.telemetry.putTrueBoolean("INPUT Intake Pinch In", intakePinchIn);
            Constants.telemetry.putTrueBoolean("INPUT Intake Pinch Out", intakePinchOut);

            Constants.telemetry.putNumber("INPUT Elbow Power (x)", towerElbowPower);
            Constants.telemetry.putNumber("INPUT Shodr Power (y)", towerShoulderPower);
            Constants.telemetry.putNumber("INPUT Wrist Power (z)", towerWristSpeed);

            Constants.telemetry.putNumber("Drive Power Offset", drivePowerOffset, true);
        }

        Constants.telemetry.putString("INPUT AUTON To Run", String.valueOf(autonToRun),true);
        Constants.telemetry.putString("INPUT AUTON Delay", String.valueOf(autonDelay),true);
        Constants.telemetry.putString("INPUT ALIANCE Color", Constants.MatchSettings.kAllianceColorName);
     */   
        

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


