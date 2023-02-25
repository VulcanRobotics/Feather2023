package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.PneumaticSubsystem;

//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.swing.plaf.synth.SynthDesktopIconUI;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.Inputs;
//import frc.robot.MyTimedPower;

//import java.time.Clock;

//import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax m_rightPinchMotor = new CANSparkMax(17, MotorType.kBrushless);
    private CANSparkMax m_leftPinchMotor = new CANSparkMax(18, MotorType.kBrushless);

    private double rightPinchSpeed = 0.0;
    private double leftPinchSpeed = 0.0;

    private double startTime = System.currentTimeMillis();
    private double elapsedtime = 0.0;
    private boolean startClock = true;
    private boolean deployTimerStarted = false;
    private double  deployStartTime = 0.0;
 
    private boolean intakeIsDeployed = false;

    public void periodic(){


        // default these here
        rightPinchSpeed = 0.0;
        leftPinchSpeed = 0.0;

        //if (m_driverXbox.getLeftTriggerAxis() >
        if(Inputs.intakeDeploy == true || Inputs.intakePinch == true || Inputs.intakePinchIn == true || Inputs.intakePinchOut == true){
            PneumaticSubsystem.setIntakeState(true);   // always Deploy in these 3 cases
            if(bIsIntakeDeployed() == false){
                startDeployTimer();         // allow intake time to deploy
     
            }else{

                if( Inputs.intakePinch == true || Inputs.intakePinchIn == true || Inputs.intakePinchOut == true){
                    
                    if( bIsIntakeDeployed() == true ){

                        PneumaticSubsystem.setPinchState(true);   // always Pinch in these 3 cases
                        if( Inputs.intakePinchIn == true || Inputs.intakePinchOut == true){

                            if (startClock == true){
                                startTime = System.currentTimeMillis();                     // cpature the current tiem in mills. 
                                startClock = false;
                                elapsedtime = 0.0;
                            }
                            else {
                                elapsedtime = System.currentTimeMillis() - startTime;       // get the total mills since we started. 
                            }

                            if( Inputs.intakePinchOut == true){
                                rightPinchSpeed = -Constants.Tower.kIntakePinchPower;   // pushing out
                                leftPinchSpeed = -rightPinchSpeed;                      // flip to go opposite of right. If right is -.5, left flips to .5
                            }
                
                            if (elapsedtime < 700) {                                        // 500 mills in .5 seconds. 
                                if( Inputs.intakePinchIn == true){
                                    rightPinchSpeed = Constants.Tower.kIntakePinchPower;    // pull in 
                                    leftPinchSpeed = -rightPinchSpeed;                      // flip to go opposite of right. If right is .5, left flips to -.5
                                }
                            }
                        }
                    }        
                    
                } else {                                                                 // no intake Pinch button spressed
                    startClock = true;
                    rightPinchSpeed = 0.0;
                    leftPinchSpeed = 0.0;
                    PneumaticSubsystem.setPinchState(false);
                }
            }

        } else {
            PneumaticSubsystem.setIntakeState(false);
            PneumaticSubsystem.setPinchState(false);
            rightPinchSpeed = 0.0;
            leftPinchSpeed = 0.0;
            intakeIsDeployed = false;
        }
        

        m_rightPinchMotor.set(rightPinchSpeed);
        m_leftPinchMotor.set(leftPinchSpeed);

        SmartDashboard.putNumber("rightPinchSpeed", rightPinchSpeed);
        SmartDashboard.putNumber("leftPinchSpeed", leftPinchSpeed);
        SmartDashboard.putNumber("elapsedTime", elapsedtime);


    }

    void startDeployTimer(){
        if (!deployTimerStarted){
            deployTimerStarted = true;
            deployStartTime = System.currentTimeMillis();
        }
    }

    boolean bIsIntakeDeployed(){
        if (System.currentTimeMillis() - deployStartTime > Constants.Tower.kIntakeDeployMills){
            deployTimerStarted = false;
            return true;
        } else{
            return false;
        }
    }

    /*void startDeployTimer(){
        // is the timer already stated and > the deploy mills?
        if( !deployTimerStarted ){
            deployTimerStarted = true;
            deployStartTime = System.currentTimeMillis();       // capture the current Mills
            intakeIsDeployed = false;
        }
    }

    /*boolean bIsIntakeDeployed(){
        if( System.currentTimeMillis() - deployStartTime >= Constants.Tower.kIntakeDeployMills )
            intakeIsDeployed = true;
            deployTimerStarted = false;

        return intakeIsDeployed;
    }*/


}
