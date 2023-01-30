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

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj.XboxController;

public class TowerSubsystem extends SubsystemBase {

    DigitalInput m_towerUpProximity = new DigitalInput(2);
    DigitalInput m_towerDownProximity = new DigitalInput(3);

    private static AnalogPotentiometer m_stringPotentiometerTower = new AnalogPotentiometer(new AnalogInput(0)); //not working 💀

    private static CANSparkMax mTower = new CANSparkMax(13, MotorType.kBrushless);
    private static CANSparkMax mElbow = new CANSparkMax(12, MotorType.kBrushless);
    private static TalonSRX m_Wrist = new TalonSRX(16);
    private static XboxController m_Controller = new XboxController(0);
    private static Joystick m_Joystick = new Joystick(1);
    private double m_encoderTower;
    private RelativeEncoder m_encoderElbow;
    private RelativeEncoder m_encoderWrist;

    private double mTowerSpeed = 0;
    private double mElbowSpeed = 0;
    private double mWristSpeed = .4;
    
    private void goToPosition(double Tvalue, double Evalue) {
        m_encoderTower = m_stringPotentiometerTower.get();
        m_encoderElbow = mElbow.getEncoder();
        if (m_encoderTower >= Tvalue + 10) {
            mTowerSpeed = -0.3;
        } else if (m_encoderTower <= Tvalue - 10) {
            mTowerSpeed = 0.3;
        } else if (m_encoderTower >= Tvalue) {
            mTowerSpeed = -0.1;
        } else if (m_encoderTower <= Tvalue) { 
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

        boolean isArmOnTop = !m_towerUpProximity.get();
        boolean isArmOnBottom = !m_towerDownProximity.get();
        
        
        
        
        //Built in encoder values 
        m_encoderElbow = mElbow.getEncoder();

        //Moving elbow
        if (Inputs.xAxisJoystick <= -0.1 || Inputs.xAxisJoystick >= 0.1){
            mElbowSpeed = m_Joystick.getX() * 0.5;
        } else {
            mElbowSpeed = 0;
        }

        //Moving shoulder
        //shaun - changed 1/25/23 - inverted y and added bottom potentiometer limit. The variable tower bottom is at the top of file.
        if (Math.abs(Inputs.yAxisJoystick) >= 0.1) { //Checking if the arm is above stringpot limit, also applying deadband
            mTowerSpeed = m_Joystick.getY() * -1; //Inverted y-axis controls 
        } else {
            mTowerSpeed = 0.0;
        }


        //For wrist movement, rotate joystick
        if (Inputs.zAxisJoystick <= -0.4 || Inputs.zAxisJoystick >= 0.4){
            mWristSpeed = 0.4;
            mWristSpeed = mWristSpeed*Inputs.zAxisJoystick;
        } else {
            mWristSpeed = 0;
        }

        //Uitilizing the goToPosition function that moves the robot to a certain position based off of the values inputted
        if (m_Joystick.getRawButton(3)) {
            goToPosition(-55, -12);
        }
        if (m_Joystick.getRawButton(4)) {
            goToPosition(33.5, 27.4);
        }

        //Limit Switches to make any last changes to the speed before setting it, must go last!
        if (isArmOnTop && mTowerSpeed > 0) {
            mTowerSpeed = 0;
        } else if (isArmOnBottom && mTowerSpeed < 0) {
            mTowerSpeed = 0;
        }
        
        mTower.set(mTowerSpeed);
        mElbow.set(mElbowSpeed);



        m_Wrist.set(ControlMode.PercentOutput, mWristSpeed);
         
        SmartDashboard.putNumber("mElbow", m_encoderElbow.getPosition());
        SmartDashboard.putBoolean("upperProximity", isArmOnTop);
        SmartDashboard.putBoolean("lowerProximity", isArmOnBottom);

        SmartDashboard.putNumber("x-axis controller", Inputs.xAxisJoystick); //Between -1 and 1
        SmartDashboard.putNumber("z-axis controller", Inputs.zAxisJoystick); 

        SmartDashboard.putNumber("String Potentiometer Tower", m_stringPotentiometerTower.get());
    }

    
}
    

