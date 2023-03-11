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

//import java.time.Clock;

//import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

public class IntakeSubsystem extends SubsystemBase {
    public static CANSparkMax m_rightPincerMotor = new CANSparkMax(17, MotorType.kBrushless);
    public static CANSparkMax m_leftPincerMotor = new CANSparkMax(18, MotorType.kBrushless);

    private XboxController m_driverXbox = Inputs.m_driverXbox;

    public static double rightPincerSpeed = 0.0;
    public static double leftPincerSpeed = 0.0;

    private double startTime = System.currentTimeMillis();
    private double elapsedtime = 0.0;
    private boolean startClock = true;

    public void autoPinch(){
        PneumaticSubsystem.setPinchState(true);
        if (startClock == true){
            startTime = System.currentTimeMillis();
            startClock = false;
            elapsedtime = 0.0;
        }
        else {
            elapsedtime = System.currentTimeMillis() - startTime;
        }
    
        if (elapsedtime < 500) {
            rightPincerSpeed = 0.5;
            leftPincerSpeed = -0.5;
        } else {
            rightPincerSpeed = 0.0;
            leftPincerSpeed = 0.0;
        }
    }

    public void periodic(){



        if (m_driverXbox.getLeftTriggerAxis() > 0.1){
            PneumaticSubsystem.setIntakeState(true);
            rightPincerSpeed = 0.5;
            leftPincerSpeed = -0.5;

            if (m_driverXbox.getRightTriggerAxis() > 0.1){
                autoPinch();
            }else if (m_driverXbox.getRightTriggerAxis() < 0.1) {
                startClock = true;
                PneumaticSubsystem.setPinchState(false);
            }

            if (Inputs.m_driverXbox.getLeftBumper()){
                rightPincerSpeed = -0.5;
                leftPincerSpeed = 0.5;
            }
            if (m_driverXbox.getRightBumper()){
                rightPincerSpeed = 0.5;
                leftPincerSpeed = -0.5;
            }

            if (!m_driverXbox.getLeftBumper() && !m_driverXbox.getRightBumper() && m_driverXbox.getRightTriggerAxis() < 0.1){
                rightPincerSpeed = 0.0;
                leftPincerSpeed = 0.0;
            }
        } else{
            PneumaticSubsystem.setIntakeState(false);
            PneumaticSubsystem.setPinchState(false);
            rightPincerSpeed = 0.0;
            leftPincerSpeed = 0.0;
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

                autoPinch();

                break;
                
            default:
                break;
        }
        
        


        m_rightPincerMotor.set(rightPincerSpeed);
        m_leftPincerMotor.set(leftPincerSpeed);

        SmartDashboard.putBoolean("Intake Deployed", PneumaticSubsystem.intakeDeployed);
        SmartDashboard.putBoolean("Pinch Closed", PneumaticSubsystem.pinchClosed);
        //SmartDashboard.putNumber("rightPincerSpeed", rightPincerSpeed);
        //SmartDashboard.putNumber("leftPincerSpeed", leftPincerSpeed);

        //SmartDashboard.putNumber("elapsedTime", elapsedtime);

    }

    public static void spinMotors() {
        m_rightPincerMotor.set(0.5);
        m_leftPincerMotor.set(-0.5);
    }
}
