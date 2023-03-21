package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.PneumaticSubsystem;
import edu.wpi.first.wpilibj.XboxController;
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
import frc.robot.Robot;

//import java.time.Clock;

//import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

public class IntakeSubsystem extends SubsystemBase {
    public static CANSparkMax m_rightPincerMotor = new CANSparkMax(17, MotorType.kBrushless);
    public static CANSparkMax m_leftPincerMotor = new CANSparkMax(18, MotorType.kBrushless);

    private XboxController m_driverXbox = Inputs.m_driverXbox;

    public static double intakeSpeed = 0.0;
    public static double leftPincerSpeed = 0.0;

    private double startTime = System.currentTimeMillis();
    private double elapsedtime = 0.0;
    private boolean startClock = true;
    private boolean locked = false;

    public void keepSpinning(){
        if (!PneumaticSubsystem.pinchClosed){
            PneumaticSubsystem.setPinchState(true);
        }
        
        if (startClock == true){
            startTime = System.currentTimeMillis();
            startClock = false;
            elapsedtime = 0.0;
        }
        else {
            elapsedtime = System.currentTimeMillis() - startTime;
        }
    
        if (elapsedtime < 1000) {
            intakeSpeed = -0.5;
            //leftPincerSpeed = 0.5;
        } else {
            intakeSpeed = 0.0;
            //leftPincerSpeed = 0.0;
        }
    }

    public void periodic(){

        if (m_driverXbox.getLeftTriggerAxis() > 0.1){
            PneumaticSubsystem.setIntakeState(true);
            intakeSpeed = 0.5;
            locked = true;
            //leftPincerSpeed = 0.5;

            //startClock = true;

            
            /*else if (m_driverXbox.getRightTriggerAxis() < 0.1) {
                startClock = true;
                //PneumaticSubsystem.setPinchState(false);
            }*/

            /*if (Inputs.m_driverXbox.getLeftBumper()){
                intakeSpeed = -0.5;
                leftPincerSpeed = 0.5;
            }
            if (m_driverXbox.getRightBumper()){
                intakeSpeed = 0.5;
                leftPincerSpeed = -0.5;
            } 

            if (!m_driverXbox.getLeftBumper() && !m_driverXbox.getRightBumper() && m_driverXbox.getRightTriggerAxis() < 0.1){
                intakeSpeed = 0.0;
                leftPincerSpeed = 0.0;
            } */
        } else /*if (Constants.MatchSettings.kInTeleop)*/ {
            PneumaticSubsystem.setIntakeState(false);
            //PneumaticSubsystem.setPinchState(false);
            //intakeSpeed = 0.0;
            //leftPincerSpeed = 0.0;
        }

        if (m_driverXbox.getRightTriggerAxis() > 0.1){
            intakeSpeed = -0.5;
            locked = false;
        }

        switch (Inputs.autonRequestIntakeGoTo){
            case IGNORE: 

                break;

            case DOWN:
                if (!PneumaticSubsystem.intakeDeployed){
                    PneumaticSubsystem.setIntakeState(true);
                }

                startClock = true;

                break;
            
            case UP:
                PneumaticSubsystem.setPinchState(false);
                PneumaticSubsystem.setIntakeState(false);
                startClock = true;

                break;

            case PINCH:
                if (PneumaticSubsystem.intakeDeployed==false){
                    PneumaticSubsystem.setIntakeState(true);
                }

                //autoPinch();

                break;
                
            default:
                break;
        }
        
        
        /*if (Inputs.rightPincerMotorSpeed != 0){leftPincerSpeed = 0.5;}
        if (Inputs.leftPincerMotorSpeed != 0){intakeSpeed = -0.5;}*/

        //keepSpinning();

        if (locked && m_driverXbox.getLeftTriggerAxis() < 0.1 && m_driverXbox.getRightTriggerAxis() < 0.1) {
            m_leftPincerMotor.set(intakeSpeed*0.25);
        } else {
            m_leftPincerMotor.set(intakeSpeed);
        }
        //m_leftPincerMotor.set(leftPincerSpeed);
        

        SmartDashboard.putBoolean("Intake Deployed", PneumaticSubsystem.intakeDeployed);
        //SmartDashboard.putBoolean("Pinch Closed", PneumaticSubsystem.pinchClosed);
        //SmartDashboard.putNumber("intakeSpeed", intakeSpeed);
        //SmartDashboard.putNumber("leftPincerSpeed", leftPincerSpeed);

        //SmartDashboard.putNumber("elapsedTime", elapsedtime);

    }

    public static void spinMotors(boolean reversed) {
        if (!reversed) {
            m_rightPincerMotor.set(-0.5);
            //m_leftPincerMotor.set(0.5);
        } else {
            m_rightPincerMotor.set(-0.5*-1);
            //m_leftPincerMotor.set(0.5*-1);
        }
    }

}

