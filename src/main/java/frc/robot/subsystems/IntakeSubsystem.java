package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.PneumaticSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.Inputs;
import frc.robot.MyTimedPower;

import java.time.Clock;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

public class IntakeSubsystem extends SubsystemBase {
    private static XboxController m_driverXbox  = new XboxController(Constants.OIConstants.kDriverControllerPort);

    private CANSparkMax m_rightPincerMotor = new CANSparkMax(17, MotorType.kBrushless);
    private CANSparkMax m_leftPincerMotor = new CANSparkMax(18, MotorType.kBrushless);

    private double rightPincerSpeed = 0.0;
    private double leftPincerSpeed = 0.0;

    private double startTime = System.currentTimeMillis();
    private double elapsedtime = 0.0;
    private boolean startClock = true;

    public void intake(){



        if (m_driverXbox.getLeftTriggerAxis() > 0.1){
            PneumaticSubsystem.toggleIntake(true);
            rightPincerSpeed = 0.5;
            leftPincerSpeed = -0.5;

            if (m_driverXbox.getRightTriggerAxis() > 0.1){
                PneumaticSubsystem.togglePinch(true);
                if (startClock == true){
                    startTime = System.currentTimeMillis();
                    startClock = false;
                    elapsedtime = 0.0;
                }
                else {
                    elapsedtime = System.currentTimeMillis() - startTime;
                }
    
                if (elapsedtime < 500) {
    //                elapsedtime = System.currentTimeMillis() - startTime;
                    rightPincerSpeed = 0.5;
                    leftPincerSpeed = -0.5;
                } else {
                    //startClock = true;
                    rightPincerSpeed = 0.0;
                    leftPincerSpeed = 0.0;
                }
    
                
            }else if (m_driverXbox.getRightTriggerAxis() < 0.1) {
                startClock = true;
                PneumaticSubsystem.togglePinch(false);
            }

            if (m_driverXbox.getLeftBumper()){
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
            PneumaticSubsystem.toggleIntake(false);
            PneumaticSubsystem.togglePinch(false);
            rightPincerSpeed = 0.0;
            leftPincerSpeed = 0.0;
        }
        

        m_rightPincerMotor.set(rightPincerSpeed);
        m_leftPincerMotor.set(leftPincerSpeed);

        SmartDashboard.putNumber("rightPincerSpeed", rightPincerSpeed);
        SmartDashboard.putNumber("leftPincerSpeed", leftPincerSpeed);

        SmartDashboard.putNumber("elapsedTime", elapsedtime);

    }
}
