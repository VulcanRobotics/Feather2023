package frc.robot;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
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

import java.sql.Driver;
import java.sql.Time;

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

    double targetQuickMove = 4000;

    double target1Piece = 100000;

    double initialPosition = 0.0;

    boolean targetLock = true;

    public  Auton() {               // constructor
        reset();
        timStepTimer.start();
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

public boolean maintainTurn(double YAWValue) {
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

                if(timStepTimer.get() < 2.45) {
                    Inputs.autonRequestTowerGoTo = AutonTowerFlags.HIGHPLACE;

                    break;
                }

          

                if (timStepTimer.get() < 3){
                    Inputs.autonRequestTowerGoTo = AutonTowerFlags.TUCKARM;
                    break;
                }
                iStep ++;
                break;

            case 2: //arm keeps to origin, starts driving to balance
                Inputs.driverPower = 0.4;

                if (timStepTimer.get() < 6.0){
                    Inputs.autonRequestTowerGoTo = AutonTowerFlags.TUCKARM;
                }
                if( DriveSubsystem.m_gyro.getRoll() < -10 ){
                    iStep++;
                    initialPosition = currentPosition;
                 }
                maintainTurn(0);
                break;

            case 3: //keeps going on balance until gyro is greater than 10
                if (bStepFirstPass) {
                    SmartDashboard.putString("Auton Step Desc", "On Ramp - Forward");
                }
                

                Inputs.driverPower = 0.25; //was 0.4

                if( DriveSubsystem.m_gyro.getRoll() > 10 ){
                    iStep++;
                    initialPosition = currentPosition;
                }


                break;
            
            case 4: //Drive a little bit forward, just to get out of the community
                if (targetPastChargeStation * 1.75 > Math.abs(currentPosition - initialPosition)) { //1.2
                    Inputs.driverPower = 0.3; 

                    
                    Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.DOWN;
                    Inputs.rightPincerMotorSpeed = 0.5;
                    Inputs.leftPincerMotorSpeed = -0.5;

                    if (timStepTimer.get() < 3.0){
                        Inputs.autonRequestTowerGoTo = AutonTowerFlags.IDLEABOVEINTAKE;
                    }

                    if (timStepTimer.get() > 0.75) {
                        maintainTurn(0);
                    }
                    
                    
                    break;
                }

                /*if (timStepTimer.get() > 1){
                    Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.PINCH;
                    break;
                }*/

                iStep++;
                break;

            case 5: //now that its out of the community, start driving backward until gyro > 10
                Inputs.driverPower = -0.3;
                Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.PINCH;

                if( DriveSubsystem.m_gyro.getRoll() > 10 ){
                    displayLightBalance();
                    iStep++;
                    initialPosition = currentPosition;
                 }


                break;
    
            case 6: //Do the same process of moving a certain distance based on ticks, but drive backward
                //displayLightBalance();
                Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.PINCH;
                if (targetPosition*2.2 - 4500 > Math.abs(currentPosition - initialPosition)) { //1.2
                    Inputs.driverPower = -0.3; //0.3 

                    break;
                }

                iStep++;
                break;

            case 7:
                if (bStepFirstPass) {
                    SmartDashboard.putString("Auton Step Desc", "Balance On Ramp");
                }
                Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.PINCH;

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
                Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.PINCH;

                if(timStepTimer.get() < 0.25) {
                    Inputs.driverTurn = -0.001;
                    break;
                } 
                //Inputs.driverPower = 0.0;
                if (timStepTimer.get() < 1.25){
                    if (targetQuickMove > Math.abs(currentPosition - initialPosition) && DriveSubsystem.m_gyro.getRoll() > 13) { //1.2
                        Inputs.driverPower = -0.3; //0.3
                    } else if (targetQuickMove > Math.abs(currentPosition - initialPosition) && DriveSubsystem.m_gyro.getRoll() < -14) {
                        Inputs.driverPower = 0.3; //0.3
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
                Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.PINCH;

                if(timStepTimer.get() < 0.25) {
                    Inputs.driverTurn = -0.001;
                    break;
                } 
                //Inputs.driverPower = 0.0;
                if (timStepTimer.get() < 1.25){
                    if (targetQuickMove > Math.abs(currentPosition - initialPosition) && DriveSubsystem.m_gyro.getRoll() > 13) { //1.2
                        Inputs.driverPower = -0.3; //0.3
                    } else if (targetQuickMove > Math.abs(currentPosition - initialPosition) && DriveSubsystem.m_gyro.getRoll() < -14) {
                        Inputs.driverPower = 0.3; //0.3
                    }
                    break;
                }
                

                iStep++;

                break;
            case 10: // turn robot 180 degrees
                if (bStepFirstPass) {
                    SmartDashboard.putString("Auton Step Desc", "Stop & Set Wheels");
                }
                Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.PINCH;
                
                Inputs.driverTurn = 0.0;
                if(timStepTimer.get() < 0.25) {
                    Inputs.driverTurn = 0.001;
                }

                if (timStepTimer.get() > 1){
                    Inputs.autonRequestTowerGoTo = AutonTowerFlags.GRABFROMINTAKE;
                }
                
                break;
            case 11: // stop robot 
                if (bStepFirstPass) {
                    SmartDashboard.putString("Auton Step Desc", "Stop & Set Wheels");
                }
                Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.PINCH;
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
                iStep ++;
                break;

            case 2: //arm keeps to origin, starts driving out of community until 7 seconds in auton

                if (timStepTimer.get() < 7.0){
                    Inputs.autonRequestTowerGoTo = AutonTowerFlags.ORIGIN;
                }
                if(timStepTimer.get() < 2.0){
                    Inputs.driverPower = 0.5;
                    break;
                }
                iStep ++;
                break;
            case 3: // stop robot from falling if unbalanced 
                if (bStepFirstPass) {
                    SmartDashboard.putString("Auton Step Desc", "Stop & Set Wheels");
                }
                Inputs.driverPower = 0.0;

                if (maintainTurn(30)) {
                    break; 
                }

                iStep++;
                break;
                
            case 4:
                if (bStepFirstPass) {
                    SmartDashboard.putString("Auton Step Desc", "Stop & Set Wheels");
                }
                maintainTurn(180);
                break;

            default:
            bIsDone = true;
            SmartDashboard.putString("Auton Step Desc", "Done");           


        }  // end of switch statement

        
    } // end of auton3 method


    public void auton4(){
        SmartDashboard.putNumber("Auton ID", 1);
        SmartDashboard.putNumber("Auton Step", iStep);
        SmartDashboard.putString("Auton Desc", "Plan to score 1 and ramp.");

        SmartDashboard.putNumber("Auton Gyro Tilt",  DriveSubsystem.m_gyro.getRoll());
        displayLightBalance();


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

                if(timStepTimer.get() < 2.45) {
                    Inputs.autonRequestTowerGoTo = AutonTowerFlags.HIGHPLACE;

                    break;
                }

          

                if (timStepTimer.get() < 3){
                    Inputs.autonRequestTowerGoTo = AutonTowerFlags.TUCKARM;
                    break;
                }
                initialPosition = currentPosition;
                iStep ++;
                break;

            case 2: //arm keeps to origin, starts driving to balance

                if (timStepTimer.get() < 6.0){
                    Inputs.autonRequestTowerGoTo = AutonTowerFlags.TUCKARM;
                }
                
                if (target1Piece*2+20000 > Math.abs(currentPosition - initialPosition)) { //1.2
                    Inputs.driverPower = 0.5; //0.3
                } else {
                    Inputs.driverPower = 0.0; //0.3
                    iStep++;
                }

                maintainTurn(0);
                
                

                break;


            case 3:
                Inputs.autonRequestTowerGoTo = AutonTowerFlags.IDLEABOVEINTAKE;
                Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.DOWN;
                if (timStepTimer.get() > 3.0){
                    Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.PINCH;
                    iStep++;
                    initialPosition = currentPosition;
                }
                maintainTurn(-45);
                break;
            case 4:
                Inputs.autonRequestTowerGoTo = AutonTowerFlags.GRABFROMINTAKE;
                
                if (target1Piece*2+20000 > Math.abs(currentPosition - initialPosition)) { //1.2
                    Inputs.driverPower = -0.5; //0.3
                } else {
                    Inputs.driverPower = 0.0; //0.3
                    iStep++;
                }

                maintainTurn(0);
                break;
            case 5:
                if (timStepTimer.get() < 3.0){
                    Inputs.driverStrafe = 0.2;
                }
                if (timStepTimer.get() > 3.0 && timStepTimer.get() < 8.0){
                    Inputs.autoCenter();
                    Inputs.autonRequestTowerGoTo = AutonTowerFlags.HIGHPLACE;
                }
                maintainTurn(0);
                break;
            case 6:
                
                
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

                if(timStepTimer.get() < 2.45) {
                    Inputs.autonRequestTowerGoTo = AutonTowerFlags.HIGHPLACE;

                    break;
                }

          

                if (timStepTimer.get() < 3){
                    Inputs.autonRequestTowerGoTo = AutonTowerFlags.TUCKARM;
                    break;
                }
                iStep ++;
                break;

            case 2: //arm keeps to origin, starts driving to balance
                Inputs.driverPower = 0.4;

                if (timStepTimer.get() < 6.0){
                    Inputs.autonRequestTowerGoTo = AutonTowerFlags.TUCKARM;
                }
                if( DriveSubsystem.m_gyro.getRoll() < -10 ){
                    iStep++;
                    initialPosition = currentPosition;
                 }

                break;

            case 3: //keeps going on balance until gyro is greater than 10
                if (bStepFirstPass) {
                    SmartDashboard.putString("Auton Step Desc", "On Ramp - Forward");
                }
                if (timStepTimer.get() < 3.0){
                    Inputs.autonRequestTowerGoTo = AutonTowerFlags.TUCKARM;
                }

                Inputs.driverPower = 0.25; //was 0.4

                if( DriveSubsystem.m_gyro.getRoll() > 10 ){
                    iStep++;
                    initialPosition = currentPosition;
                 }


                break;
            
            case 4: //Drive a little bit forward, just to get out of the community
                if (targetPastChargeStation * 1.75 > Math.abs(currentPosition - initialPosition)) { //1.2
                    Inputs.driverPower = 0.3; 

                    
                    Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.DOWN;
                    Inputs.rightPincerMotorSpeed = 0.5;
                    Inputs.leftPincerMotorSpeed = -0.5;

                    if (timStepTimer.get() > 0.75) {
                        maintainTurn(0);
                    }
                    
                    
                    break;
                }

                /*if (timStepTimer.get() > 1){
                    Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.PINCH;
                    break;
                }*/

                iStep++;
                break;

            case 5: //now that its out of the community, start driving backward until gyro > 10
                Inputs.driverPower = -0.4;
                Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.PINCH;

                if( DriveSubsystem.m_gyro.getRoll() > 10 ){
                    displayLightBalance();
                    iStep++;
                    initialPosition = currentPosition;
                 }


                break;
    
            case 6: //Do the same process of moving a certain distance based on ticks, but drive backward
                //displayLightBalance();
                Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.PINCH;
                if (targetPosition*2.2 - 4500 > Math.abs(currentPosition - initialPosition)) { //1.2
                    Inputs.driverPower = -0.3; //0.3 

                    break;
                }

                iStep++;
                break;

            case 7:
                if (bStepFirstPass) {
                    SmartDashboard.putString("Auton Step Desc", "Balance On Ramp");
                }
                Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.PINCH;

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
            
                
                iStep++;

                break;
            case 9: // drive back forward to get a good shot.
        
                if (bStepFirstPass) {
                    SmartDashboard.putString("Auton Step Desc", "Balance On Ramp");
                }
                Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.PINCH;

                if(timStepTimer.get() < 0.25) {
                    Inputs.driverTurn = -0.001;
                    break;
                } 
                //Inputs.driverPower = 0.0;
                if (DriveSubsystem.m_gyro.getYaw()>3){
                    Inputs.driverPower = -m_balancePID.calculate(Math.abs(DriveSubsystem.m_gyro.getRoll()), 2.0);
                    break;
                }
                

                iStep++;

                break;
            case 10: // turn robot 180 degrees
                if (bStepFirstPass) {
                    SmartDashboard.putString("Auton Step Desc", "Stop & Set Wheels");
                }
                Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.PINCH;
                
                Inputs.driverTurn = 0.0;
                if(timStepTimer.get() < 0.25) {
                    Inputs.driverTurn = 0.001;
                } else {
                    iStep++;
                }
                
                break;
            case 11: // stop robot 
                if (bStepFirstPass) {
                    SmartDashboard.putString("Auton Step Desc", "Stop & Set Wheels");
                }
                Inputs.autonRequestIntakeGoTo = AutonIntakeFlags.PINCH;
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

    




} // end of auton class
