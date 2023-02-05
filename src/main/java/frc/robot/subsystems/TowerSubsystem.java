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
    DigitalInput m_ElbowDownProximity = new DigitalInput(8);
    DigitalInput m_ElbowUpProximity = new DigitalInput(7);


    private static AnalogPotentiometer m_stringPotentiometerTower = new AnalogPotentiometer(new AnalogInput(0));
    private static AnalogPotentiometer m_stringPotentiometerElbow = new AnalogPotentiometer(new AnalogInput(1));

    private static CANSparkMax mTower = new CANSparkMax(13, MotorType.kBrushless);
    private static CANSparkMax mElbow = new CANSparkMax(12, MotorType.kBrushless);
    private static TalonSRX m_Wrist = new TalonSRX(16);
    private static XboxController m_Controller = new XboxController(0);
    private static Joystick m_Joystick = new Joystick(1);
    private RelativeEncoder m_encoderTower;
    private RelativeEncoder m_encoderElbow;
    private RelativeEncoder m_encoderWrist;

    private double m_stringTower = 0.0;
    private double m_stringElbow = 0.0;

    private double mTowerSpeed = 0;
    private double mElbowSpeed = 0;
    private double mWristSpeed = .4;

    private boolean IKMode = false;

    private double targetX = 0.6;
    private double targetY = 0.74;

    private double theta1 = 0.0;
    private double theta2 = 0.0;
     
    private void goToPosition(double Tvalue, double Evalue) {
        m_stringTower = m_stringPotentiometerTower.get();
        m_stringElbow = m_stringPotentiometerElbow.get();
        if (m_stringTower >= Tvalue + .01) {
            mTowerSpeed = 0.9;
        } else if (m_stringTower <= Tvalue - .01) {
            mTowerSpeed = -0.9;
        } else if (m_stringTower >= Tvalue + 0.001) {
            mTowerSpeed = 0.3;
        } else if (m_stringTower <= Tvalue - 0.001) { 
            mTowerSpeed = -0.3;
        }
        if (m_stringElbow >= Evalue + .01) {
            mElbowSpeed = -0.3;
        } else if (m_stringElbow <= Evalue - .01) {
            mElbowSpeed = 0.3;
        } else if (m_stringElbow >= Evalue + 0.001) {
            mElbowSpeed = -0.1;
        } else if (m_stringElbow <= Evalue - 0.001) {
            mElbowSpeed = 0.1;
        }

    }

    private void tuckArm(){
        if (m_towerDownProximity.get() == true){
            mTowerSpeed = 0.75;
        }
        if (m_stringPotentiometerElbow.get() > 0.58){
            mElbowSpeed = -0.15;
        } else if(m_stringPotentiometerElbow.get() < 0.565){
            mElbowSpeed = 0.15;
        }
    }

    public void tower(){

        boolean isArmOnTop = !m_towerUpProximity.get();
        boolean isArmOnBottom = !m_towerDownProximity.get();
        
        boolean isElbowOnTop = m_ElbowUpProximity.get();
        boolean isElbowOnBottom = !m_ElbowDownProximity.get();
        
        //Built in encoder values 
        m_encoderElbow = mElbow.getEncoder();
        m_encoderTower = mTower.getEncoder();

        double encoderElbow = m_encoderElbow.getPosition();
        double encoderTower = m_encoderTower.getPosition();

        if (m_Joystick.getRawButtonPressed(4)){
            IKMode = !IKMode;
        }
        //Moving elbow

        if (IKMode == false){
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
        } //else
        
            if (Math.abs(Inputs.xAxisJoystick) >= 0.1) { // Dead stick zone
                targetX += m_Joystick.getX() * 0.1; // Very small changes, for now
                if (targetX < 0.1) { // Hard limit for now to keep things forward (positive X)
                    targetX = 0.1;
                }
            }
            if (Math.abs(Inputs.yAxisJoystick) >= 0.1) { // Dead stick zone
                targetY += m_Joystick.getY() * 0.1; // Very small changes, for now
            }


            double z1 = 0.4826; //meters
            double z2 = 0.3683; //meters
            double y0 = 0.69; // tower height in meters

            double f = 2*targetX;
            double h = 2*targetY;
            double k = z2*z2;
            double m = targetX*targetX + targetY*targetY + k - z1*z1;

            double a = f*f + h*h;
            double b = -2*m*f;
            double c = m*m - (h*h*k);

            double x2p = (-b + Math.sqrt(b*b - 4*a*c)) / (2*a);      
            double x2n = (-b - Math.sqrt(b*b - 4*a*c)) / (2*a);

            double x2 = x2n;
            double x1 = targetX - x2;

            double y2 = Math.sqrt(z2*z2 - x2*x2);
            double y1 = targetY - y2 - y0;


            SmartDashboard.putNumber("a", a);
            SmartDashboard.putNumber("b", b);            
            SmartDashboard.putNumber("c", c);
            SmartDashboard.putNumber("x1", x1);
            SmartDashboard.putNumber("x2", x2);            
            SmartDashboard.putNumber("x2n", x2n);
            SmartDashboard.putNumber("x2p", x2p);
            SmartDashboard.putNumber("y1", y1);
            SmartDashboard.putNumber("y2", y2);

            theta1 = ( Math.atan(y1/x1) );
            theta2 = ( Math.atan(y2/x2) );
        
            //theta1 = Math.toDegrees( Math.atan(y1/x1) );
            //theta2 = Math.toDegrees( Math.atan(y2/x2) );


        //For wrist movement, rotate joystick
        if (Inputs.zAxisJoystick <= -0.4 || Inputs.zAxisJoystick >= 0.4){
            mWristSpeed = 0.4;
            mWristSpeed = mWristSpeed*Inputs.zAxisJoystick;
        } else {
            mWristSpeed = 0;
        }

        //Uitilizing the goToPosition function that moves the robot to a certain position based off of the values inputted
        if (m_Joystick.getRawButton(3)) {
            //goToPosition(-55, -12);
            tuckArm();
        }
        if (m_Joystick.getRawButton(5)) {
            goToPosition(0.185, 0.519);
        }

        //Limit Switches to make any last changes to the speed before setting it, must go last!
        if (isArmOnTop && mTowerSpeed > 0) {
            mTowerSpeed = 0;
        } else if (isArmOnBottom && mTowerSpeed < 0) {
            mTowerSpeed = 0;
        }
        if (isElbowOnTop && mElbowSpeed < 0) { //changed from towerspeed to elbowspeed
            mElbowSpeed = 0;
        } else if (isElbowOnBottom && mElbowSpeed > 0) { //changed from towerspeed to elbowspeed
            mElbowSpeed = 0;
        }
        
       /*  if (m_stringPotentiometerTower.get() >= 0.48 && m_stringPotentiometerTower.get() <= 0.53){
            if (m_stringPotentiometerElbow.get() >= 0.53){
                mElbowSpeed = -0.1;
            }
        }*/

        mTower.set(mTowerSpeed);
        mElbow.set(mElbowSpeed);



        m_Wrist.set(ControlMode.PercentOutput, mWristSpeed);
         
        SmartDashboard.putNumber("mElbow", m_stringPotentiometerElbow.get());
        SmartDashboard.putBoolean("upperProximity", isElbowOnTop);
        SmartDashboard.putBoolean("lowerProximity", isElbowOnBottom);
        SmartDashboard.putBoolean("Elbow down Prox", m_ElbowDownProximity.get());

        SmartDashboard.putNumber("mElbowSpeed", mElbowSpeed);

        SmartDashboard.putNumber("x-axis controller", Inputs.xAxisJoystick); //Between -1 and 1
        SmartDashboard.putNumber("z-axis controller", Inputs.zAxisJoystick); 

        SmartDashboard.putNumber("String Potentiometer Tower", m_stringPotentiometerTower.get());
        SmartDashboard.putNumber("String Potentiometer Elbow", m_stringPotentiometerElbow.get());

        SmartDashboard.putNumber("Tower Encoder", encoderTower);
        SmartDashboard.putNumber("Elbow Encoder", encoderElbow);
        
        SmartDashboard.putNumber("Tower IK Angle", theta1);
        SmartDashboard.putNumber("Elbow IK Angle", theta2);

        SmartDashboard.putNumber("Target X", targetX);
        SmartDashboard.putNumber("Target Y", targetY);




        //TESTING SOMETHING
    }

    
}
    

