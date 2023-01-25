package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Inputs;

//import frc.robot.LinearServo;
import frc.robot.RampPower;

import edu.wpi.first.wpilibj.Joystick;

import java.util.Map;

import javax.lang.model.util.ElementScanner6;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj.XboxController;

public class TowerSubsystem extends SubsystemBase {
    
    private static CANSparkMax mTower = new CANSparkMax(13, MotorType.kBrushless);
    private static CANSparkMax mElbow = new CANSparkMax(12, MotorType.kBrushless);
    private static TalonSRX m_Wrist = new TalonSRX(16);
    private static XboxController m_Controller = new XboxController(0);
    private static Joystick m_Joystick = new Joystick(1);
    private RelativeEncoder m_encoderTower;
    private RelativeEncoder m_encoderElbow;
    private RelativeEncoder m_encoderWrist;
    
    private double mTowerSpeed = 0;
    private double mElbowSpeed = 0;
    private double mWristSpeed = .4;

    private boolean armModeToggle = false;
    

    private void goToPosition(double Tvalue, double Evalue) {
        m_encoderTower = mTower.getEncoder();
        m_encoderElbow = mElbow.getEncoder();

        if (m_encoderTower.getPosition() >= Tvalue + 10) {
            mTowerSpeed = -0.3;
        } else if (m_encoderTower.getPosition() <= Tvalue - 10) {
            mTowerSpeed = 0.3;
        } else if (m_encoderTower.getPosition() >= Tvalue) {
            mTowerSpeed = -0.1;
        } else if (m_encoderTower.getPosition() <= Tvalue) { 
            mTowerSpeed = 0.1;
        }



        if (m_encoderElbow.getPosition() >= Evalue + 10) {
            mElbowSpeed = -0.3;
        } else if (m_encoderElbow.getPosition() <= Evalue - 10) {
            mElbowSpeed = 0.3;
        } else if (m_encoderElbow.getPosition() >= Evalue) {
            mElbowSpeed = -0.1;
        } else if (m_encoderElbow.getPosition() <= Evalue) {
            mElbowSpeed = 0.1;
        }

    }

    public void tower(){
        m_encoderTower = mTower.getEncoder();
        m_encoderElbow = mElbow.getEncoder();
        
        if (m_Joystick.getRawButtonPressed(2)){
            armModeToggle = !armModeToggle;
        }

        //For large arm

        if(m_Joystick.getRawButton(7)) {
            mTowerSpeed = -0.3;
        }
        else if(m_Joystick.getRawButton(8)) {
            mTowerSpeed = 0.3;
        }
        else {
            mTowerSpeed = 0;
        }

        //for elbow arm
        if(m_Joystick.getRawButton(9)) {
            mElbowSpeed = -0.3;
        }
        else if(m_Joystick.getRawButton(10)) {
            mElbowSpeed = 0.3;
        }
        else {
            mElbowSpeed = 0;
        }

        if (armModeToggle){
            mTowerSpeed = 0.0;
            mElbowSpeed = m_Joystick.getY() * 0.5;
        }else{
            mElbowSpeed = 0.0;
            mTowerSpeed = m_Joystick.getY() * 0.5;
        }

        if (m_encoderTower.getPosition() <= -57 && mTowerSpeed <= 0) {
            mTowerSpeed = 0;
        } else if (m_encoderTower.getPosition() >= 36 && mTowerSpeed >= 0) {
            mTowerSpeed = 0;
        } 

        if (m_Joystick.getRawButton(3)) {
            goToPosition(-55, -12);
        }
        if (m_Joystick.getRawButton(4)) {
            goToPosition(33.5, 27.4);
        }

        if (Inputs.xAxisJoystick <= -0.1 || Inputs.xAxisJoystick >= 0.1){
            mWristSpeed = mWristSpeed*Inputs.xAxisJoystick;
        } else {
            mWristSpeed = 0;
        }
        //SmartDashboard.getNumber("mTower", mTower.get());
        //SmartDashboard.getNumber("mElbow", mElbow.get());
        mTower.set(mTowerSpeed);
        mElbow.set(mElbowSpeed);
        m_Wrist.set(ControlMode.PercentOutput, mWristSpeed);
        mWristSpeed = .4;


        SmartDashboard.putNumber("mTower", m_encoderTower.getPosition());
        SmartDashboard.putNumber("mElbow", m_encoderElbow.getPosition());

        SmartDashboard.putNumber("x-axis controller", Inputs.xAxisJoystick); //Between -1 and 1

    }

    
}
    

