package frc.robot;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.Tower;
import frc.robot.Constants.Tower.AutonIntakeFlags;
import frc.robot.Constants.Tower.AutonTowerFlags;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.TimedRampPower;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanEntry;

import java.sql.Driver;
import java.sql.Time;

import org.xml.sax.InputSource;

import com.kauailabs.navx.frc.AHRS;

/* Auton Class
	This is custom version of autonomous operation. As you can see it extends the StateMachine class so it will can take
    on all the StateMachine's charactistics. Usign this you can design up to 10 autonomous state machines to do run
    during the autonomous period for the robot.

 */


public class Auton extends MyStateMachine {

    // Auton class constructor. We are passing several other classes that we need access to get use their FBW values.
    // We are passing references for these classes.

    String status = "";
    String stepDesc = "";

	private Timer timShootingLimit = new Timer();	// used to decide if we should keep shooting

    private ProfiledPIDController m_balancePID = new ProfiledPIDController(0.01, 0.0,0.0, new TrapezoidProfile.Constraints(0.5, 1));
	
    private TimedRampPower trpDrivePower;
    


    double targetPosition = 26000;  //ticks
    double targetPastChargeStation = 52000;

    double targetQuickMove = 7000; //4000

    double target1Piece = 100000;

    double initialPosition = 0.0;

    boolean targetLock = true;

    public  Auton() {               // constructor
        reset();
        timStepTimer.start();
    }

    private double flaconToFeet(double ticks) {
        double temp = Units.metersToFeet(MathUtil.falconToMeters(ticks, 24.76, 6.75));
        return temp;
    }

    private double feetToFalcon(double feet) {
        double temp = MathUtil.metersToFalcon(Units.feetToMeters(feet), 24.76, 6.75);
        return temp;
    }

public void displayLightBalance() {
        if (Math.abs(DriveSubsystem.m_gyro.getRoll()) < 2.5){
            Inputs.redLED = true;

        }else if(Math.abs(DriveSubsystem.m_gyro.getRoll()) > 2.5){
            Inputs.greenLED = true;
        } else{
            Inputs.redLED = true;
        }
    }

public boolean maintainTurn(double YAWValue, boolean ignoreDeadBand) {
    if (!ignoreDeadBand){
        if (Math.abs(Inputs.currentYAW - Math.abs(YAWValue)) > 2) {
            if (YAWValue == 180 || YAWValue == -180){
                if (Inputs.currentYAW < 0.0) {
                    Inputs.driverTurn = -0.2;
                }
                if (Inputs.currentYAW > 0.0) {
                    Inputs.driverTurn = 0.2;
                }
            } else {
                if (Inputs.currentYAW > YAWValue+30) {
                    Inputs.driverTurn = -0.3;
                } else if (Inputs.currentYAW < YAWValue-30) {
                    Inputs.driverTurn = 0.3;
                } else if (Inputs.currentYAW > YAWValue+2) {
                    Inputs.driverTurn = -0.1;
                } else if (Inputs.currentYAW < YAWValue-2) {
                    Inputs.driverTurn = 0.1;
                }
            }
            return false;
        } else {
            return true;
        }
    } else{
        if (Inputs.currentYAW > YAWValue) {
            Inputs.driverTurn = -0.1;
        } else if (Inputs.currentYAW < YAWValue) {
            Inputs.driverTurn = 0.1;
        }

        return false;
    }
    

    
    
}

    public void auton1() { // This overrides the auton2 method defined in the state machine class.
        
        Inputs.redLED = true;
        //displayLightBalance();
        //DriveSubsystem.m_gyro.zeroYaw();
        //String sAuton = "Auton1 -  ";
        //SmartDashboard.putString("Auton ", sAuton );

        //SmartDashboard.saveDouble("Auton Step Timer", timStepTimer.get() );  // capture timer on ever step

        SmartDashboard.putNumber("Auton ID", 1);
        SmartDashboard.putNumber("Auton Step", iStep);
        SmartDashboard.putString("Auton Desc", "Plan to score 1 and ramp.");

        SmartDashboard.putNumber("Auton Gyro Tilt",  DriveSubsystem.m_gyro.getRoll());

        if( bStepFirstPass)                                         // no need to do this in every                                                          
            timStepTimer.reset();                                   // reset the timer on a step change

        Inputs.fieldCentric = true;        // do this in call cases

        double currentPosition = DriveSubsystem.m_frontLeft.getCurrentDriveTicks();

        SmartDashboard.putNumber("Current Position", currentPosition);

        switch (iStep) {                            // switch statement.
            // it will look at iStep and find the case where it should run code.
            // if iStep not found, it will go to default section at bottom.

            

            case 0:              
                Inputs.driveSwerveEncoderReset = true;                      // case 0 is where we can set up a bunch of stuff for the state machine.    
                if (bStepFirstPass) {
                    SmartDashboard.putString("Auton Step Desc", "Delay Test");
                    DriveSubsystem.m_gyro.zeroYaw();

                }

                if (Inputs.autonDelay == 0) {
                    iStep += 1;
                } else if (timStepTimer.get() >= Inputs.autonDelay) {
                    iStep += 1;
                }
                //iStep = 2; //if only trying to do charge station

                break;

            
            /*case 1: // drive back to wall with intake down
                Inputs.driveSwerveEncoderReset = false;
            if (bStepFirstPass) {
                SmartDashboard.putString("Auton Step Desc", "Attack Ramp");
            }

                if(timStepTimer.get() < 3.5) {
                    Inputs.autonRequestTowerGoTo = AutonTowerFlags.HIGHPLACE;

                    break;
                }

          

                if (timStepTimer.get() < 7.0){
                    Inputs.autonRequestTowerGoTo = AutonTowerFlags.ORIGIN;
                    break;
                }

                iStep ++;
                break;

            case 2:
                Inputs.driverPower = 1;

                if (timStepTimer.get() < 6.0){
                    Inputs.autonRequestTowerGoTo = AutonTowerFlags.ORIGIN;
                }
                if( DriveSubsystem.m_gyro.getRoll() < -10 ){
                    displayLightBalance();
                    iStep++;
                    initialPosition = currentPosition;
                 }

                break;

            case 3: 
                //displayLightBalance();
            
                if (bStepFirstPass) {
                    SmartDashboard.putString("Auton Step Desc", "On Ramp - Forward");
                }

                
                
                if (targetPosition*3 - 18000 > Math.abs(currentPosition - initialPosition)) { //1.2
                    Inputs.driverPower = 0.3; //0.3

                    break;
                }

                

                iStep++;


                break;
            
            case 4:
                if (bStepFirstPass) {
                    SmartDashboard.putString("Auton Step Desc", "Balance On Ramp");
                }

                Inputs.driverPower = 0.0;
                if(timStepTimer.get() < 0.25) {
                    Inputs.driverTurn = .00001;
                }

                initialPosition = currentPosition;
                if(timStepTimer.get() > 2) {
                    iStep++;
                }
                break;
            case 5: // drive back forward to get a good shot.
                if (bStepFirstPass) {
                    SmartDashboard.putString("Auton Step Desc", "Balance On Ramp");
                }

                //displayLightBalance();

                

                if (targetQuickMove > Math.abs(currentPosition - initialPosition) && DriveSubsystem.m_gyro.getRoll() > 2.5) { //1.2
                    Inputs.driverPower = -0.3; //0.3
                    break;
                } else if (targetQuickMove > Math.abs(currentPosition - initialPosition) && DriveSubsystem.m_gyro.getRoll() < -2.5) {
                    Inputs.driverPower = 0.3; //0.3
                    break;
                }

                iStep++;

                break;
            case 6: // stop robot 
                if (bStepFirstPass) {
                    SmartDashboard.putString("Auton Step Desc", "Stop & Set Wheels");
                }
                Inputs.driverPower = 0.0;
                if(timStepTimer.get() < 0.25) {
                    Inputs.driverTurn = .00001;
                    iStep++;
                }
                
                break;
            
                */
            default:
                bIsDone = true;
                SmartDashboard.putString("Auton Step Desc", "Done");           


        }  // end of switch statement                                                        
    }



    public void auton2() { // Same thing as auton1, but now we leave the community for a short time for the additional 3 points
        
        Inputs.redLED = true;

        //displayLightBalance();
        //DriveSubsystem.m_gyro.zeroYaw();
        //String sAuton = "Auton1 -  ";
        //SmartDashboard.putString("Auton ", sAuton );

        //SmartDashboard.saveDouble("Auton Step Timer", timStepTimer.get() );  // capture timer on ever step

        SmartDashboard.putNumber("Auton ID", 1);
        SmartDashboard.putNumber("Auton Step", iStep);
        SmartDashboard.putString("Auton Desc", "Plan to score 1 and ramp.");

        SmartDashboard.putNumber("Auton Gyro Tilt",  DriveSubsystem.m_gyro.getRoll());

        if( bStepFirstPass)                                         // no need to do this in every                                                          
            timStepTimer.reset();                                   // reset the timer on a step change

        Inputs.fieldCentric = true;        // do this in call cases

        double currentPosition = DriveSubsystem.m_frontLeft.getCurrentDriveTicks();

        SmartDashboard.putNumber("Current Position", currentPosition);

        //Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.IGNORE;

        switch (iStep) {                            // switch statement.
            // it will look at iStep and find the case where it should run code.
            // if iStep not found, it will go to default section at bottom.

            

            case 0:              
                Inputs.driveSwerveEncoderReset = true;                      // case 0 is where we can set up a bunch of stuff for the state machine.    
                if (bStepFirstPass) {
                    SmartDashboard.putString("Auton Step Desc", "Delay Test");
                    DriveSubsystem.m_gyro.zeroYaw();
                    
                }

                if (Inputs.autonDelay == 0) {
                    iStep += 1;
                } else if (timStepTimer.get() >= Inputs.autonDelay) {
                    iStep += 1;
                }
                //iStep = 2; //if only trying to do charge station

                break;

            
            case 1: //Put the cone down on the high place
                Inputs.driveSwerveEncoderReset = false;
            if (bStepFirstPass) {
                SmartDashboard.putString("Auton Step Desc", "Attack Ramp");
            }

                if(!Inputs.armReachedTarget/*timStepTimer.get() < 2.45*/) {
                    Inputs.autonRequestTowerGoTo = AutonTowerFlags.HIGHPLACE;

                    break;
                }

          

                if (timStepTimer.get() < 2){
                    Inputs.autonRequestTowerGoTo = AutonTowerFlags.ORIGIN;
                    break;
                }
                iStep ++;
                break;

            case 2: //arm keeps to origin, starts driving to balance
                Inputs.driverPower = -0.4;

                if (timStepTimer.get() < 6.0){
                    Inputs.autonRequestTowerGoTo = AutonTowerFlags.ORIGIN;
                }
                if( DriveSubsystem.m_gyro.getRoll() < -10 ){
                    iStep++;
                    initialPosition = currentPosition;
                 }
                //maintainTurn(0, false);
                break;

            case 3: //keeps going on balance until gyro is greater than 10
                if (bStepFirstPass) {
                    SmartDashboard.putString("Auton Step Desc", "On Ramp - Forward");
                }
                
                Inputs.autonRequestTowerGoTo = AutonTowerFlags.ORIGIN;
                Inputs.driverPower = -0.25; //was 0.4
                maintainTurn(0, false);

                if( DriveSubsystem.m_gyro.getRoll() > 10 ){
                    iStep++;
                    initialPosition = currentPosition;
                }


                break;
            
            case 4: //Drive a little bit forward, just to get out of the community
                if (targetPastChargeStation * 2 > Math.abs(currentPosition - initialPosition)) { //1.2
                    Inputs.driverPower = -0.3; 

                    
                    Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.INTAKE;

                    //IntakeSubsystem.spinMotors();
                    if (timStepTimer.get() < 3.0){
                        Inputs.autonRequestTowerGoTo = AutonTowerFlags.ORIGIN;
                    }

                    if (timStepTimer.get() > 0.75) {
                        Inputs.seekTarget();
                    } /*else if (timStepTimer.get() < 0.75) {
                        maintainTurn(0, false);
                    }*/        
                    
                    break;
                }

                /*if (timStepTimer.get() > 1){
                    Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.PINCH;
                    break;
                }*/

                iStep++;
                break;

            case 5: //now that its out of the community, start driving backward until gyro > 10
                Inputs.driverPower = 0.5;
                //Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.PINCH;

                maintainTurn(0, false);

                /*if (timStepTimer.get() < 1.0){
                    Inputs.rightPincerMotorSpeed = 0.5;
                    Inputs.leftPincerMotorSpeed = -0.5;
                }*/

                Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.UP;

                if (timStepTimer.get() > 1){
                    Inputs.autonRequestTowerGoTo = AutonTowerFlags.ORIGIN;
                }



                if( DriveSubsystem.m_gyro.getRoll() > 10 ){
                    displayLightBalance();
                    iStep++;
                    initialPosition = currentPosition;
                 }


                break;
    
            case 6: //Do the same process of moving a certain distance based on ticks, but drive backward
                //displayLightBalance();
                if ((targetPosition*2.2 - 500)*0.9 > Math.abs(currentPosition - initialPosition)) { //1.2 // was -4500
                    Inputs.driverPower = 0.3; //0.3 

                    break;
                }


                iStep++;
                break;

            case 7:
                if (bStepFirstPass) {
                    SmartDashboard.putString("Auton Step Desc", "Balance On Ramp");
                }
                initialPosition = currentPosition;
                Inputs.driverPower = 0.0;
                if(timStepTimer.get() < 0.25) {
                    Inputs.driverTurn = 0.0001;
                } 

                if(timStepTimer.get() > 1) {
                    iStep++;
                    Inputs.driverPower = 0.0;
                }
                break;
            case 8: // drive back forward to get a good shot.
            
                if (bStepFirstPass) {
                    SmartDashboard.putString("Auton Step Desc", "Balance On Ramp");
                }
                if(timStepTimer.get() < 0.25) {
                    Inputs.driverTurn = -0.001;
                    break;
                } 
                //Inputs.driverPower = 0.0;
                if (timStepTimer.get() < 1.25){
                    if (targetQuickMove > Math.abs(currentPosition - initialPosition) && DriveSubsystem.m_gyro.getRoll() > 13) { //1.2
                        Inputs.driverPower = 0.3; //0.3
                    } else if (targetQuickMove > Math.abs(currentPosition - initialPosition) && DriveSubsystem.m_gyro.getRoll() < -13) {
                        Inputs.driverPower = -0.3; //0.3
                    }
                    break;
                }
                
                initialPosition = currentPosition;
                iStep++;

                break;
            case 9: // drive back forward to get a good shot.
        
                if (bStepFirstPass) {
                    SmartDashboard.putString("Auton Step Desc", "Balance On Ramp");
                }
                if(timStepTimer.get() < 0.25) {
                    Inputs.driverTurn = -0.001;
                    break;
                } 
                //Inputs.driverPower = 0.0;
                if (timStepTimer.get() < 1.25){
                    if (targetQuickMove-3000 > Math.abs(currentPosition - initialPosition) && DriveSubsystem.m_gyro.getRoll() > 10) { //1.2
                        Inputs.driverPower = 0.3; //0.3
                    } else if (targetQuickMove-3000 > Math.abs(currentPosition - initialPosition) && DriveSubsystem.m_gyro.getRoll() < -10) {
                        Inputs.driverPower = -0.3; //0.3
                    }
                    break;
                }
                

                iStep++;

                break;
            case 10: // turn robot 180 degrees
                if (bStepFirstPass) {
                    SmartDashboard.putString("Auton Step Desc", "Stop & Set Wheels");
                }                
                Inputs.driverTurn = 0.0;
                if(timStepTimer.get() < 0.25) {
                    Inputs.driverTurn = 0.001;
                }

                /*if (timStepTimer.get() > 1){
                    Inputs.autonRequestTowerGoTo = AutonTowerFlags.GRABFROMINTAKE;
                }*/
                
                break;
            case 11: // stop robot 
                if (bStepFirstPass) {
                    SmartDashboard.putString("Auton Step Desc", "Stop & Set Wheels");
                }
                Inputs.driverPower = 0.0;
                if(timStepTimer.get() < 0.25) {
                    Inputs.driverTurn = 0.001;
                }
                
                break;
            default:
            bIsDone = true;
            SmartDashboard.putString("Auton Step Desc", "Done");           


        }  // end of switch statement                                                         
    }

    /*public void auton6() { // Our test Auton
        String sAuton = "Auton6 - Play Auton Test";
        SmartDashboard.putString("Auton ", sAuton );

        if(Robot.pbAutonTest.bMomentsLoaded==true && Robot.pbAutonTest.isPlaybackDone()==false)
            Robot.pbAutonTest.playNextMoment();

    }*/


    public void auton3(){
        SmartDashboard.putNumber("Auton ID", 1);
        SmartDashboard.putNumber("Auton Step", iStep);
        SmartDashboard.putString("Auton Desc", "Plan to score 1 and ramp.");

        SmartDashboard.putNumber("Auton Gyro Tilt",  DriveSubsystem.m_gyro.getRoll());

        Inputs.redLED = true;

        //displayLightBalance();

        if( bStepFirstPass)                                         // no need to do this in every                                                          
            timStepTimer.reset();                                   // reset the timer on a step change

        Inputs.fieldCentric = true;        // do this in call cases

        double currentPosition = DriveSubsystem.m_frontLeft.getCurrentDriveTicks();

        SmartDashboard.putNumber("Current Position", currentPosition);

        switch (iStep) {                            // switch statement.
            // it will look at iStep and find the case where it should run code.
            // if iStep not found, it will go to default section at bottom.

            

            case 0:              
                Inputs.driveSwerveEncoderReset = true;                      // case 0 is where we can set up a bunch of stuff for the state machine.    
                if (bStepFirstPass) {
                    SmartDashboard.putString("Auton Step Desc", "Delay Test");
                    DriveSubsystem.m_gyro.zeroYaw();
                }

                if (Inputs.autonDelay == 0) {
                    iStep += 1;
                } else if (timStepTimer.get() >= Inputs.autonDelay) {
                    iStep += 1;
                }
                //iStep = 2; //if only trying to do charge station

                break;

            
            case 1: //Put the cone down on the high place
                Inputs.driveSwerveEncoderReset = false;
            if (bStepFirstPass) {
                SmartDashboard.putString("Auton Step Desc", "Attack Ramp");
            }

                if(timStepTimer.get() < 3.5) {
                    Inputs.autonRequestTowerGoTo = AutonTowerFlags.HIGHPLACE;

                    break;
                }

          

                if (timStepTimer.get() < 7.0){
                    Inputs.autonRequestTowerGoTo = AutonTowerFlags.ORIGIN;
                    break;
                } 
                initialPosition = currentPosition;
                iStep ++;
                break;

            case 2: //arm keeps to origin, starts driving out of community until 7 seconds in auton

                if (timStepTimer.get() < 7.0){
                    Inputs.autonRequestTowerGoTo = AutonTowerFlags.ORIGIN;
                }
                if (feetToFalcon(4) > Math.abs(currentPosition - initialPosition)) { //1.2
                    Inputs.driverPower = 0.2; //0.3
                    Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.DOWN;
                } else {
                    Inputs.driverPower = 0.0;
                    iStep++;
                }
                iStep ++;
                break;
            case 3: // stop robot from falling if unbalanced 
                if (bStepFirstPass) {
                    SmartDashboard.putString("Auton Step Desc", "Stop & Set Wheels");
                }
                Inputs.driverPower = 0.0;

                if (maintainTurn(30, false)) {
                    break; 
                }

                iStep++;
                break;
                
            case 4:
                if (bStepFirstPass) {
                    SmartDashboard.putString("Auton Step Desc", "Stop & Set Wheels");
                }
                break;

            default:
            bIsDone = true;
            SmartDashboard.putString("Auton Step Desc", "Done");           


        }  // end of switch statement

        
    } // end of auton3 method


    public void auton4(int reflectionFactor){
        SmartDashboard.putNumber("Auton ID", 1);
        SmartDashboard.putNumber("Auton Step", iStep);
        SmartDashboard.putString("Auton Desc", "Plan to score 1 and ramp.");
        

        SmartDashboard.putNumber("Auton Gyro Tilt",  DriveSubsystem.m_gyro.getRoll());
        displayLightBalance();


        if( bStepFirstPass)                                         // no need to do this in every                                                          
            timStepTimer.reset();                                   // reset the timer on a step change

        Inputs.fieldCentric = true;        // do this in call cases

        double currentPosition = DriveSubsystem.m_frontLeft.getCurrentDriveTicks();
        SmartDashboard.putNumber("Distance from Init Possy", currentPosition - initialPosition);
        SmartDashboard.putNumber("Current Position", currentPosition);
        SmartDashboard.putNumber("Initial Position", initialPosition);

        switch (iStep) {                            // switch statement.
            // it will look at iStep and find the case where it should run code.
            // if iStep not found, it will go to default section at bottom.

            

            case 0:              
                Inputs.driveSwerveEncoderReset = true;                      // case 0 is where we can set up a bunch of stuff for the state machine.    
                if (bStepFirstPass) {
                    SmartDashboard.putString("Auton Step Desc", "Delay Test");
                    DriveSubsystem.m_gyro.zeroYaw();

                }

                if (Inputs.autonDelay == 0) {
                    iStep += 1;
                } else if (timStepTimer.get() >= Inputs.autonDelay) {
                    iStep += 1;
                }
                //iStep = 2; //if only trying to do charge station

                break;

            
            case 1: //Put the cone down on the high place
                Inputs.driveSwerveEncoderReset = false;
            if (bStepFirstPass) {
                SmartDashboard.putString("Auton Step Desc", "Attack Ramp");
            }

                if(timStepTimer.get() < 3) {
                    Inputs.autonRequestTowerGoTo = AutonTowerFlags.HIGHPLACE;

                    break;
                }

          

                /*if (timStepTimer.get() < 1){
                    Inputs.autonRequestTowerGoTo = AutonTowerFlags.ORIGIN;
                    break;
                }*/
                initialPosition = currentPosition;
                iStep++;
                break;

            case 2: //arm keeps to origin, starts driving to balance

                Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.INTAKE;
                //Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.INTAKE;

                Inputs.autonRequestTowerGoTo = AutonTowerFlags.ORIGIN;
                
                /*if (target1Piece*2 > Math.abs(currentPosition - initialPosition)) { 
                    Inputs.driverPower = -0.4; //0.3
                    maintainTurn(15, false);
                } else if (!Inputs.seekTarget() || timStepTimer.get() < 3.0) {
                    Inputs.driverPower = -0.2;
                } else {
                    iStep++;
                }*/

                if (timStepTimer.get() < 1.5){
                    Inputs.driverPower = -0.5;
                    maintainTurn(reflectionFactor * 0, false);
                } else {
                    if (!Inputs.seekTarget()) {
                        Inputs.driverPower = -0.5;
                    } else {
                        iStep++;
                    }
                }
                 //-35
                
                

                break;


            case 3:
                Inputs.autonRequestTowerGoTo = AutonTowerFlags.ORIGIN;

                Inputs.seekTarget();

                Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.INTAKE;
                


                iStep++;
                initialPosition = currentPosition;
                break;
            case 4:
                
                
                if (timStepTimer.get() > 1.0){
                    Inputs.autonRequestTowerGoTo = AutonTowerFlags.GRABFROMINTAKE;
                    
                }
                
                Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.UP;
                
                if (target1Piece*2 -5000 > Math.abs(currentPosition - initialPosition)) { //1.2
                    Inputs.driverPower = 0.3; //0.3
                } else {
                    Inputs.driverPower = 0.0; //0.3
                    iStep++;
                }

                maintainTurn(-3, false);

               /*  if (timStepTimer.get() > 3){
                    Inputs.autoCenter(1);
                } */
                
                break;
            case 5:
                Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.GRABINTAKE;
                if (timStepTimer.get() < 0.6) {
                    Inputs.driverStrafe = -0.27;
                    
                } else 
                if (timStepTimer.get() < 2.0){
                    Inputs.autoCenter(1);
                } if (timStepTimer.get() >1.5 && timStepTimer.get() <2.5){
                    Inputs.driverPower = 0.2;
                }

                 if (timStepTimer.get() < 4.0){
                   
                    Inputs.autonRequestTowerGoTo = AutonTowerFlags.HIGHPLACE;
                }
                if (timStepTimer.get() > 4.0){
                    iStep++;
                    initialPosition = currentPosition;
                }

                maintainTurn(0, false);
                break;
            case 6:
                
                //Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.INTAKE;

                /* 
                if (timStepTimer.get() < 2.0){
                    maintainTurn(60, false);
                } else {
                    Inputs.seekTarget();
                } */
                if (timStepTimer.get() < 0.5) {
                    Inputs.driverStrafe = 0.5;
                    Inputs.autonRequestTowerGoTo = AutonTowerFlags.ORIGIN;
                    break;
                }


                /*if (target1Piece*2 > Math.abs(currentPosition - initialPosition)) { //1.2
                    Inputs.driverPower = -0.8; //0.3
                    
                } else if (target1Piece*2+75000 > Math.abs(currentPosition - initialPosition)){
                    Inputs.driverPower = -0.2;
                    iStep++;
                }*/
                /* 
                if (DriveSubsystem.m_gyro.getYaw() < 40) { //1.2
                    Inputs.driverPower = -0.8; //0.3
                    
                } else if (DriveSubsystem.m_gyro.getYaw() < 65){
                    Inputs.driverPower = -0.2;
                } else {
                    iStep++;
                } */

                /*if (timStepTimer.get() < 0.6){
                    Inputs.driverStrafe = -0.3;
                }*

                if (timStepTimer.get() > 2.5){
                    Inputs.driverStrafe = 0.3;
                }*/
        
               iStep++;
                
                break;
            
            case 7:
               /*  Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.INTAKE;
                if (!Inputs.seekTarget()){
                    Inputs.driverStrafe = 0.3;
                } else {
                    iStep++;
                    initialPosition = currentPosition;
                }

                Inputs.autonRequestTowerGoTo = AutonTowerFlags.ORIGIN;
            */
            if (timStepTimer.get() < 1.5) {
                Inputs.autonRequestTowerGoTo = AutonTowerFlags.ORIGIN;
                Inputs.driverPower  = -0.8;
                maintainTurn(0, false);

            }
                break;
            /*case 8:

                if (10000 > Math.abs(currentPosition - initialPosition)){
                    Inputs.driverStrafe = -0.4;
                } else {
                   iStep++; 
                   initialPosition = currentPosition;
                }

                
                if (timStepTimer.get() > 2.0){
                    Inputs.autonRequestTowerGoTo = AutonTowerFlags.GRABFROMINTAKE;
                }
                Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.UP;

                break;
                
            case 9:

                if (timStepTimer.get() > 2.0){
                    Inputs.autonRequestTowerGoTo = AutonTowerFlags.GRABFROMINTAKE;
                }
                Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.UP;

                Inputs.autoCenter(1);

                if (target1Piece*2-10000 > Math.abs(currentPosition - initialPosition)) { //1.2
                    Inputs.driverPower = 0.4; //0.3
                } else {
                    Inputs.driverPower = 0.0; //0.3
                    if (timStepTimer.get() < 2.0){
                        Inputs.driverStrafe = -0.7;
                    } else {
                        iStep++;
                        Inputs.driverStrafe = 0.0;
                    }
                    
                    
                }

                break;*/

            default:
            bIsDone = true;
            SmartDashboard.putString("Auton Step Desc", "Done");           


        }  // end of switch statement
        
    } // end of auton4 method

    public void auton5() { // Same thing as auton1, but now we leave the community for a short time for the additional 3 points
        
        Inputs.redLED = true;

        //displayLightBalance();
        //DriveSubsystem.m_gyro.zeroYaw();
        //String sAuton = "Auton1 -  ";
        //SmartDashboard.putString("Auton ", sAuton );

        //SmartDashboard.saveDouble("Auton Step Timer", timStepTimer.get() );  // capture timer on ever step

        SmartDashboard.putNumber("Auton ID", 1);
        SmartDashboard.putNumber("Auton Step", iStep);
        SmartDashboard.putString("Auton Desc", "Plan to score 1 and ramp.");

        SmartDashboard.putNumber("Auton Gyro Tilt",  DriveSubsystem.m_gyro.getRoll());

        if( bStepFirstPass)                                         // no need to do this in every                                                          
            timStepTimer.reset();                                   // reset the timer on a step change

        Inputs.fieldCentric = true;        // do this in call cases

        double currentPosition = DriveSubsystem.m_frontLeft.getCurrentDriveTicks();

        SmartDashboard.putNumber("Current Position", currentPosition);
        

        switch (iStep) {                            // switch statement.
            // it will look at iStep and find the case where it should run code.
            // if iStep not found, it will go to default section at bottom.

            

            case 0:              
                Inputs.driveSwerveEncoderReset = true;                      // case 0 is where we can set up a bunch of stuff for the state machine.    
                if (bStepFirstPass) {
                    SmartDashboard.putString("Auton Step Desc", "Delay Test");
                    DriveSubsystem.m_gyro.zeroYaw();
                }

                if (Inputs.autonDelay == 0) {
                    iStep += 1;
                } else if (timStepTimer.get() >= Inputs.autonDelay) {
                    iStep += 1;
                }
                //iStep = 2; //if only trying to do charge station

                break;

            
            case 1: //Put the cone down on the high place
                Inputs.driveSwerveEncoderReset = false;
            if (bStepFirstPass) {
                SmartDashboard.putString("Auton Step Desc", "Attack Ramp");
            }

                Inputs.autonRequestTowerGoTo = AutonTowerFlags.GRABFROMINTAKE;
                
                break;

            default:

            break;
        }     
    }


} // end of auton class
