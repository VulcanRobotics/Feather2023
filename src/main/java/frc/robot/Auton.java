package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
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
	
    private TimedRampPower trpDrivePower;

    public  Auton() {               // constructor
        reset();
        timStepTimer.start();       // required as MyStateMachine cannot do this.
    }

    public void auton1() { // This overrides the auton2 method defined in the state machine class.
        //DriveSubsystem.m_gyro.zeroYaw();
        //String sAuton = "Auton1 -  ";
        //Constants.telemetry.putString("Auton ", sAuton );

        //Constants.telemetry.saveDouble("Auton Step Timer", timStepTimer.get() );  // capture timer on ever step

        Constants.telemetry.putNumber("Auton ID", 1, true);
        Constants.telemetry.putNumber("Auton Step", iStep, true);
        Constants.telemetry.putString("Auton Desc", "Plan to score 1 and ramp.", true);

        Constants.telemetry.putNumber("Auton Gyro Tilt",  DriveSubsystem.m_gyro.getRoll(), true);

        if( bStepFirstPass)                                         // no need to do this in every                                                          
            timStepTimer.reset();                                   // reset the timer on a step change

        Inputs.fieldCentric = true;        // do this in call cases

        switch (iStep) {                            // switch statement.
            // it will look at iStep and find the case where it should run code.
            // if iStep not found, it will go to default section at bottom.

            case 0:                                    // case 0 is where we can set up a bunch of stuff for the state machine.    
                if (bStepFirstPass) {
                    Constants.telemetry.putString("Auton Step Desc", "Delay Test", true);
                    DriveSubsystem.m_gyro.zeroYaw();
                }

                if (Inputs.autonDelay == 0) {
                    iStep += 1;
                } else if (timStepTimer.get() >= Inputs.autonDelay) {
                    iStep += 1;
                }

                break;

            
            case 1: // drive back to wall with intake down
            if (bStepFirstPass) {
                Constants.telemetry.putString("Auton Step Desc", "Attack Ramp", true);
            }

                Inputs.driverPower = .8;

                if( DriveSubsystem.m_gyro.getRoll() < -10 )
                    iStep++; //++;

                break;

            case 2: 
                if (bStepFirstPass) {
                    Constants.telemetry.putString("Auton Step Desc", "On Ramp - Forward", true);
                }

                Inputs.driverPower = .2;       

                if( DriveSubsystem.m_gyro.getRoll() >= -5.0 )
                    iStep = 4; //++;

                break;

            case 3: // drive back forward to get a good shot.
                if (bStepFirstPass) {
                    Constants.telemetry.putString("Auton Step Desc", "Balance On Ramp", true);
                }

                Inputs.driverPower = 0;

                if (timStepTimer.get() < 0.25){
                    Inputs.driverTurn = 0.1;
                    iStep++;
                }

                break;
    
            case 4: // stop robot 
                if (bStepFirstPass) {
                    Constants.telemetry.putString("Auton Step Desc", "Stop & Set Wheels", true);
                }

                Inputs.driverPower = 0.0;

                if (timStepTimer.get() < 0.25){
                    Inputs.driverTurn = 0.1;
                    iStep++;
                }

                break;

            default:
                bIsDone = true;
                Constants.telemetry.putString("Auton Step Desc", "Done" , true);           


        }  // end of switch statement                                                        
    }



    public void auton2() { // Basic backup, get a ball, drive up and shoot on the Left tarmac
        DriveSubsystem.m_gyro.zeroYaw();
        String sAuton = "Auton2 - 2 Ball ";
        Constants.telemetry.putString("Auton ", sAuton, true );
        Constants.telemetry.putNumber("Auton Step Timer", timStepTimer.get(), true );  // capture timer on ever step

        if( bStepFirstPass){                                         // no need to do this in every                                                          
            timStepTimer.reset();                                   // reset the timer on a step change
        }
        
        switch (iStep) {                            // switch statement.
            // it will look at iStep and find the case where it should run code.
            // if iStep not found, it will go to default section at bottom.

            case 0:                                    // case 0 is where we can set up a bunch of stuff for the state machine.    
                if (bStepFirstPass) {
                }

                /*iStep++;
                break;*/

                if (Inputs.autonDelay == 0) {
                    iStep += 1;
                } else if (timStepTimer.get() >= Inputs.autonDelay) {
                    iStep += 1;
                }

                break;
            
            /* case 1: // drive back to a certain spot for 4.0 seconds
                if (bStepFirstPass) {

                    Constants.telemetry.saveString("Auton Step Desc", status );           
                    trpDrivePower.reconfigure(Constants.Auton2BallConstants.kInitial_Time, 
                                              -Constants.Auton2BallConstants.kInitial_Power, 
                                              -.1,  // min drive power
                                              .10,  // ramp up percent, .20 as we have to reset gyro here
                                              .20);

                }
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "Started, rolling back to ball, intake delpoyed.";

                //Inputs.driveDesiredHeading = 0.0;       
                //Inputs.driveGyroCorrected = true;       // force robot to turn an maintain this heading

                Inputs.intakeDeploy = true;             // drop the intake
                Inputs.shooterFullAutoModeOn = false;    // turn on targetting
                Inputs.shooterFullAutoModeOff = true;    // turn on targetting

                Inputs.driverPower = trpDrivePower.getRampedPower(timStepTimer.get());
                
                if( trpDrivePower.isDone())             // the 
                    iStep = 3; //++;

                Constants.telemetry.saveString("Auton Step Desc", status );           
                break;

            case 2: // drive back to a certain spot for 4.0 seconds
                if (bStepFirstPass) {
                    status = "Step " + String.valueOf(iStep) + ": First Pass...";
                    Constants.telemetry.saveString("Auton Step Desc", status );           
                    trpDrivePower.reconfigure(Constants.Auton2BallConstants.kToShoot_Time, 
                                            Constants.Auton2BallConstants.kToShoot_Power, 
                                             -.1,  // min drive power
                                            .10,  // ramp up percent, .20 as we have to reset gyro here
                                            .20);

                    break;
                }
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "Started, rolling back to start Pos, intake retracted.";

                Inputs.driveDesiredHeading = Constants.Auton2BallConstants.kToShoot_Heading;       
                //Inputs.driveGyroCorrected = true;       // force robot to turn an maintain this heading
                Inputs.driveGyroCorrected = false; //Zach change today

                Inputs.intakeDeploy = false;             // drop the intake
                Inputs.shooterFullAutoModeOn = true;    // turn on targetting
                Inputs.shooterFullAutoModeOff = false;    // turn on targetting

                Inputs.driverPower = trpDrivePower.getRampedPower(timStepTimer.get());
                
                if( trpDrivePower.isDone())             // the 
                    iStep++;

                Constants.telemetry.saveString("Auton Step Desc", status );           
                break;


            case 3: // Stop and shoot
            if (bStepFirstPass) {
            }
            status = "Step " + String.valueOf(iStep) + ": ";
            status += "Shooting";

            //Inputs.driveDesiredHeading = 0.0;       
                //Inputs.driveGyroCorrected = false;              // tuen this off in case we are thrashing

                Inputs.intakeDeploy = false;                    // drop the intake
                Inputs.shooterFullAutoModeOn = true;            // turn on targetting
                Inputs.shooterFullAutoModeOff = false;          

                Inputs.operatorTrigger = true;

                if( timStepTimer.get() > 5.0 )
                    iStep++;

                Constants.telemetry.saveString("Auton Step Desc", status );           
                break;
            */
            default:
                bIsDone = true;
                //Inputs.shooterFullAutoModeOff = true;            // turn on targetting
                Constants.telemetry.putString("Auton Step Desc", "Done", true );           


        }  // end of switch statement                                                        
    }

    public void auton5() { // This overrides the auton1 method defined in the state machine class.
        String sAuton = "Auton5 - Play Auton 3 ball";
        Constants.telemetry.putString("Auton ", sAuton );

        //if(Robot.pbAuton3ball.bMomentsLoaded==true && Robot.pbAuton3ball.isPlaybackDone()==false)
         //   Robot.pbAuton3ball.playNextMoment();

    }

    /*public void auton6() { // Our test Auton
        String sAuton = "Auton6 - Play Auton Test";
        Constants.telemetry.putString("Auton ", sAuton );

        if(Robot.pbAutonTest.bMomentsLoaded==true && Robot.pbAutonTest.isPlaybackDone()==false)
            Robot.pbAutonTest.playNextMoment();

    }*/


    public void auton3(){
        String sAuton = "Auton3 - 3 Ball Right ";
        Constants.telemetry.putString("Auton ", sAuton );

        Constants.telemetry.saveDouble("Auton Step Timer", timStepTimer.get() );  // capture timer on ever step

        if( bStepFirstPass)                                         // no need to do this in every                                                          
            timStepTimer.reset();                                   // reset the timer on a step change

        //Inputs.fieldCentric = false;        // do this in call cases

        switch (iStep) {                            // switch statement.
            // it will look at iStep and find the case where it should run code.
            // if iStep not found, it will go to default section at bottom.

            case 0:                                    // case 0 is where we can set up a bunch of stuff for the state machine.    
                if (bStepFirstPass) {
                }

                iStep++;
                break;

            /* 
            case 1: // drive back to wall where ball is
                if (bStepFirstPass) {
                    trpDrivePower.reconfigure(Constants.Auton3BallRightConstants.kInitial_Time, 
                                              -Constants.Auton3BallRightConstants.kInitial_Power, 
                                              -.1,  // min drive power
                                              .20,  // ramp up percent, .20 as we have to reset gyro here
                                              .20);
                }
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "Started, rolling back to ball, intake delpoyed.";
                
                //Inputs.driveDesiredHeading = 0.0;       
                //Inputs.driveGyroCorrected = true;      // don use at this point // force robot to turn an maintain this heading

                Inputs.intakeDeploy = true;             // drop the intake
                Inputs.shooterFullAutoModeOn = true;    // turn on targetting

                Inputs.driverPower = trpDrivePower.getRampedPower(timStepTimer.get());
                
                if( trpDrivePower.isDone())             // the 
                    iStep++;

                Constants.telemetry.saveString("Auton Step Desc", status );           
    
                break;

            case 2: // drive back to spot I started at
                if (bStepFirstPass) {
                    trpDrivePower.reconfigure(Constants.Auton3BallRightConstants.kToShoot_Time, 
                                              -Constants.Auton3BallRightConstants.kToShoot_Power, 
                                              -.1,  // min drive power
                                              .20,  // ramp up percent, .20 as we have to reset gyro here
                                              .20);
                }
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "Started, rolling back to ball, intake delpoyed.";
                
                //Inputs.driveDesiredHeading = 0.0;       
                //Inputs.driveGyroCorrected = true;      // don use at this point // force robot to turn an maintain this heading

                Inputs.intakeDeploy = false;             // drop the intake
                Inputs.shooterFullAutoModeOn = true;    // turn on targetting

                Inputs.driverPower = trpDrivePower.getRampedPower(timStepTimer.get());
                
                if( trpDrivePower.isDone())             // the 
                    iStep++;

                Constants.telemetry.saveString("Auton Step Desc", status );           
    
                break;

            case 3: // Shoot
                if (bStepFirstPass) {
					  timShootingLimit.reset();	// reset this itmer
                }
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "Shoot!";
                //Inputs.driveDesiredHeading = Constants.Auton4BallConstants.kToTerm_Heading; 
                //Inputs.driveGyroCorrected = false;       // force robot to turn an maintain this heading

                Inputs.intakeDeploy = false;             // pick up intake
                Inputs.shooterFullAutoModeOn = true;    // turn on targetting

                Inputs.operatorTrigger = true;
                Inputs.operatorThumbSafety = true;

                /*if( IntakeSubsystem.ball1IsReady() ||          // if there is a ball in the shooter
                      IntakeSubsystem.ball2IsReady() )
					  timShootingLimit.reset();						// reset this timer

                                                                   
                Constants.telemetry.saveString("Auton Step Desc", status );           

                break;

            case 4:                                         // drive to a the terminal at 20 degrees
                if (bStepFirstPass) {

                    trpDrivePower.reconfigure(Constants.Auton3BallRightConstants.kToBall3_Time, 
                                                -Constants.Auton3BallRightConstants.kToBall3_Power,
                                               .1,  // this direction will be adjusted to the max power direction 
                                              .10, 
                                              .30);  // .50 is ramp down 50 percent before the end
                }
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "Going to Ball 3, slewing turret left.";
    
                Inputs.driveDesiredHeading = Constants.Auton3BallRightConstants.kToBall3_Heading; 
                Inputs.driveGyroCorrected = true; 
                // other stuff you wnat in here
                Inputs.driverPower = trpDrivePower.getRampedPower(timStepTimer.get());
                Inputs.shooterFullAutoModeOn = false;    // turn on targetting
                Inputs.intakeDeploy = true;

                //Inputs.turretDesiredPosit = Constants.Auton3BallConstants.kTo3Ball_TurretPosit; 

                if( trpDrivePower.isDone())
                    iStep++;

                Constants.telemetry.saveString("Auton Step Desc", status );           
                break; 
				

            case 5: // Taking the third ball shot
                if (bStepFirstPass) {

                }
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "Shooting ball 3.";
        
                Inputs.operatorTrigger = true;                
                Inputs.shooterFullAutoModeOn = true;    // turn on targetting
                Inputs.intakeDeploy = false;

                Constants.telemetry.saveString("Auton Step Desc", status );           
                break;
            */

            default: 
                Inputs.driverPower = 0.0;
                //Inputs.shooterFullAutoModeOff = true;    // turn on targetting

            }   // end of switch/case statement
        
    } // end of auton2 method


    public void auton4(){
        String sAuton = "Auton4 - 4 Ball ";
        Constants.telemetry.putString("Auton ", sAuton );

        Constants.telemetry.saveDouble("Auton Step Timer", timStepTimer.get() );  // capture timer on ever step

        if( bStepFirstPass)                                         // no need to do this in every                                                          
            timStepTimer.reset();                                   // reset the timer on a step change

        //Inputs.fieldCentric = false;        // do this in call cases

        switch (iStep) {                            // switch statement.
            // it will look at iStep and find the case where it should run code.
            // if iStep not found, it will go to default section at bottom.

            case 0:                                    // case 0 is where we can set up a bunch of stuff for the state machine.    
                if (bStepFirstPass) {
                }

                iStep++;
                break;

         /*/   case 1: // drive back to a certain spot for 4.0 seconds
                if (bStepFirstPass) {
                    trpDrivePower.reconfigure(Constants.Auton4BallConstants.kInitial_Time, 
                                              Constants.Auton4BallConstants.kInitial_Power, 
                                              .10,  // min drive power, max power sets direction
                                              .10,  // ramp up percent, .20 as we have to reset gyro here
                                              .10);
                }
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "Started, rolling back to ball, intake delpoyed.";
                
                Inputs.fieldCentric = false;
                Inputs.driveDesiredHeading = 0.0;       
                Inputs.driveGyroCorrected = true;      // don use at this point // force robot to turn an maintain this heading

                Inputs.intakeDeploy = true;             // drop the intake
                Inputs.shooterFullAutoModeOn = true;    // turn on targetting

                Inputs.driverPower = trpDrivePower.getRampedPower(timStepTimer.get());
                
                if( trpDrivePower.isDone())             // the 
                    iStep++;

                Constants.telemetry.saveString("Auton Step Desc", status );           
    
                break;


            case 2: // shoot
                if (bStepFirstPass) {
					  timShootingLimit.reset();	// reste this itmer
                }
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "Shoot!";
                //Inputs.driveDesiredHeading = Constants.Auton4BallConstants.kToTerm_Heading; 
                //Inputs.driveGyroCorrected = false;       // force robot to turn an maintain this heading

                Inputs.intakeDeploy = false;             // pick up intake
                Inputs.shooterFullAutoModeOn = true;    // turn on targetting

                Inputs.operatorTrigger = true;
                Inputs.operatorThumbSafety = true;

                // we waited long enough
                    iStep++;                                          // next step, 
                                                                        
                Constants.telemetry.saveString("Auton Step Desc", status );           

                break;

             case 3:   // back up after the shot
                if (bStepFirstPass) {

                    trpDrivePower.reconfigure(Constants.Auton4BallConstants.kAfterShot1_Time, 
                                                Constants.Auton4BallConstants.kAfterShot1_Power,
                                               .1,  // this direction will be adjusted to the max power direction 
                                              .10, 
                                              .10);  // .50 is ramp down 50 percent before the end
                }
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "Bakc up more/";
    
                Inputs.driveDesiredHeading = Constants.Auton4BallConstants.kToTerm_Heading; 
                Inputs.driveGyroCorrected = true; 
                // other stuff you wnat in here
                Inputs.driverPower = trpDrivePower.getRampedPower(timStepTimer.get());
                Inputs.shooterFullAutoModeOff = true;    // turn off targetting
                Inputs.intakeDeploy = false;

                if( trpDrivePower.isDone())
                    iStep++;   // stop here

                Constants.telemetry.saveString("Auton Step Desc", status );           
                break; 
             

            case 3:   // Traverse to Terminal and third ball                                     
                if (bStepFirstPass) {

                    trpDrivePower.reconfigure(Constants.Auton4BallConstants.kToTerm_Time, 
                                                Constants.Auton4BallConstants.kToTerm_Power,
                                               .1,  // this direction will be adjusted to the max power direction 
                                              .10, 
                                              .30);  // .50 is ramp down 50 percent before the end
                }
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "To Terminal";
    
                Inputs.driveDesiredHeading = Constants.Auton4BallConstants.kToTerm_Heading; 
                Inputs.driveGyroCorrected = true; 
                // other stuff you wnat in here

                // Here we are traversing
                Inputs.driverStrafe = trpDrivePower.getRampedPower(timStepTimer.get());
                Inputs.shooterFullAutoModeOff = true;    // turn off targetting
                Inputs.intakeDeploy = false;

                if( trpDrivePower.isDone())
                    iStep++;   // stop here

                Constants.telemetry.saveString("Auton Step Desc", status );           
                break; 

            case 4:   // drive back to the terminal, collect first ball
                if (bStepFirstPass) {
                    trpDrivePower.reconfigure(Constants.Auton4BallConstants.kAtTerm_Time, 
                                                Constants.Auton4BallConstants.kAtTerm_Power,
                                                -.1, 
                                                .10, 
                                                Constants.Auton4BallConstants.kAtTerm_RampDownProp);  // .50 is ramp down 50 percent before the end
                }
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "Drive up to terminal";
    
                Inputs.driveDesiredHeading = Constants.Auton4BallConstants.kAtTerm_Heading; 
                Inputs.driveGyroCorrected = true; 
                // other stuff you wnat in here
                Inputs.driverPower = trpDrivePower.getRampedPower(timStepTimer.get());
                Inputs.shooterFullAutoModeOff = true;    // turn on targetting
                Inputs.intakeDeploy = true;
                
                if( trpDrivePower.isDone()){
                    iStep++;
                }

                break;

            case 5:   // Should have the ball, raise the intake
                if (bStepFirstPass) {
                }
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "At Teminal, raise intake";
    
                Inputs.driverPower = .3;                 // turn off targetting
                Inputs.shooterFullAutoModeOff = true;    // turn off targetting
                Inputs.intakeDeploy = false;
                
                if( timStepTimer.get() > Constants.Auton4BallConstants.kAtTerm_IntakeUpTime){
                    iStep++;
                }

                break;


            case 6:   // Human player rolls ball under, drop the intake
                if (bStepFirstPass) {
                }
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "At Teminal, lower intake";
    
                Inputs.shooterFullAutoModeOn = true;    // turn on targetting
                Inputs.intakeDeploy = true;
                
                if( timStepTimer.get() > Constants.Auton4BallConstants.kAtTerm_IntakeDownTime){
                    iStep++;
                }

                break;



            case 7: // drive To shoot
                if (bStepFirstPass) {
                    trpDrivePower.reconfigure(Constants.Auton4BallConstants.kShot2_DriveTime, 
                                        Constants.Auton4BallConstants.kShot2_Power,
                                        -.1, 
                                        .10, 
                                        .40);  // .50 is ramp down 50 percent before the end

                }
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "Shoot!";
        
                Inputs.driveDesiredHeading = Constants.Auton4BallConstants.kShot2_Heading; 
                Inputs.driveGyroCorrected = true; 
                Inputs.shooterFullAutoModeOn = true;    // turn on targetting
                Inputs.intakeDeploy = false;
                
                Inputs.driverPower = trpDrivePower.getRampedPower(timStepTimer.get());
 
                if( trpDrivePower.isDone() )
                    iStep++;

                Constants.telemetry.saveString("Auton Step Desc", status );           
                break; 
    

            case 8: // Taking the final shot
                if (bStepFirstPass) {

                }
                status = "Step " + String.valueOf(iStep) + ": ";
                Inputs.shooterFullAutoModeOn = true;    // turn on targetting

                if(timStepTimer.get() < Constants.Auton4BallConstants.kShot2_DelayShotTime){
                    status += "Delaying the Shot to allow robot to stop forward momentum!";
                } else {
                    Inputs.operatorTrigger = true;                
                    status += "Shooting!!!";
                }

                //Inputs.operatorTrigger = true;

                Constants.telemetry.saveString("Auton Step Desc", status );           
                // no need to move to a new step as shootingn is the last thing we do in auton
                break; 
                
            */

            default: 
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "Done";
                Inputs.driverPower = 0.0;
                Constants.telemetry.saveString("Auton Step Desc", status );           

            }   // end of switch/case statement
        
    } // end of auton4 method


} // end of auton class
