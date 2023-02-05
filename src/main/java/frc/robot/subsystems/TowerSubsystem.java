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

    private double targetX = 0.85;
    private double targetY = 0.69;

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
        }
        //else {
        
            if (Math.abs(Inputs.xAxisJoystick) >= 0.1) { // Dead stick zone
                targetX += m_Joystick.getX() * 0.1; // Very small changes, for now
                if (targetX < 0.1) { // Hard limit for now to keep things forward (positive X)
                    targetX = 0.1;
                }
            }
            if (Math.abs(Inputs.yAxisJoystick) >= 0.1) { // Dead stick zone
                targetY += m_Joystick.getY() * 0.1; // Very small changes, for now
                if (targetY > 1.0) {
                    targetY = 1.0;
                }
            }

            double z1 = 0.4826; //meters
            double z2 = 0.3683; //meters
            double y0 = 0.69; // tower height in meters
            double x0 = 0.0;
            double PI = 3.1415926535;

            double tower_rad = PI*(m_stringPotentiometerTower.get() - 0.099595)/0.11;
            double elbow_rad = -PI*(m_stringPotentiometerElbow.get() - 0.63)/0.131;
            double tow_y = Math.sin(tower_rad) * z1;
            double tow_x = Math.cos(tower_rad) * z1;
            double elb_y = Math.sin(elbow_rad) * z2;
            double elb_x = Math.cos(elbow_rad) * z2;

            SmartDashboard.putNumber("Tower X", tow_x);
            SmartDashboard.putNumber("Tower Y", tow_y);   
            SmartDashboard.putNumber("Tower angle", 180*(tower_rad)/PI );          
            SmartDashboard.putNumber("Elbow X", elb_x);
            SmartDashboard.putNumber("Elbow Y", elb_y);
            SmartDashboard.putNumber("Elbow angle", 180*(elbow_rad)/PI );            


            double k = (targetX-x0)*(targetX-x0) + (targetY-y0)*(targetY-y0) + z1*z1 - z2*z2;

            double a = 4*((targetX-x0)*(targetX-x0) + (targetY-y0)*(targetY-y0));
            double b = -4*(targetY-y0)*k;
            double c = k*k - 4*(targetX-x0)*(targetX-x0) * z1*z1;

            double y1pp = (-b + Math.sqrt(b*b - 4*a*c)) / (2*a);      
            double y1pn = (-b + Math.sqrt(b*b - 4*a*c)) / (2*a);      
            double y1np = (-b - Math.sqrt(b*b - 4*a*c)) / (2*a);
            double y1nn = (-b - Math.sqrt(b*b - 4*a*c)) / (2*a);

            double x1pp = Math.sqrt(z1*z1 - y1pp*y1pp);
            double x1pn = -Math.sqrt(z1*z1 - y1pn*y1pn);
            double x1np = Math.sqrt(z1*z1 - y1np*y1np);
            double x1nn = -Math.sqrt(z1*z1 - y1nn*y1nn);

            double x2pp = targetX - x1pp - x0;
            double x2pn = targetX - x1pn - x0;
            double x2np = targetX - x1np - x0;
            double x2nn = targetX - x1nn - x0;

            double y2pp = targetY - y1pp - y0;
            double y2pn = targetY - y1pn - y0;
            double y2np = targetY - y1np - y0;
            double y2nn = targetY - y1nn - y0;

            double z1pp = Math.sqrt(x1pp*x1pp + y1pp*y1pp);
            double z1pn = Math.sqrt(x1pn*x1pn + y1pn*y1pn);
            double z1np = Math.sqrt(x1np*x1np + y1np*y1np);
            double z1nn = Math.sqrt(x1nn*x1nn + y1nn*y1nn);
            
            double z2pp = Math.sqrt(x2pp*x2pp + y2pp*y2pp);
            double z2pn = Math.sqrt(x2pn*x2pn + y2pn*y2pn);
            double z2np = Math.sqrt(x2np*x2np + y2np*y2np);
            double z2nn = Math.sqrt(x2nn*x2nn + y2nn*y2nn);

            double x1fa = 0.0, x1fb = 0.0, x1f=0.0;
            double y1fa = 0.0, y1fb = 0.0, y1f=0.0;
            double x2fa = 0.0, x2fb = 0.0, x2f=0.0;
            double y2fa = 0.0, y2fb = 0.0, y2f=0.0;

            if (Math.abs(z1pp - z1) < 0.01 && Math.abs(z2pp - z2) < 0.01) {
                x1fa = x1pp; y1fa = y1pp;
                x2fa = x2pp; y2fa = y2pp;
            }
            else {
                x1fa = x1pn; y1fa = y1pn;
                x2fa = x2pn; y2fa = y2pn;
            }

            if (Math.abs(z1pn - z1) < 0.01 && Math.abs(z2pn - z2) < 0.01) {
                x1fb = x1pn; y1fb = y1pn;
                x2fb = x2pn; y2fb = y2pn;
            }
            else {
                x1fb = x1nn; y1fb = y1nn;
                x2fb = x2nn; y2fb = y2nn;
            }

            //theta1 = Math.atan2(y1fa,x1fa);
            //theta2 = Math.atan2(y2fa,x2fa);

            double theta1a = Math.atan2(y1fa,x1fa);
            double theta2a = Math.atan2(y2fa,x2fa);
            double theta1b = Math.atan2(y1fb,x1fb);
            double theta2b = Math.atan2(y2fb,x2fb);
        
            if ((Math.abs(theta1a-tower_rad) + Math.abs(theta2a-elbow_rad)) < (Math.abs(theta1b-tower_rad) + Math.abs(theta2b-elbow_rad))) {
                theta1 = theta1a; theta2 = theta2a;
            }
            else {
                theta1 = theta1b; theta2 = theta2b;
            }

            SmartDashboard.putNumber("a", a);
            SmartDashboard.putNumber("b", b);            
            SmartDashboard.putNumber("c", c);
            SmartDashboard.putNumber("x1", x1f);
            SmartDashboard.putNumber("x2", x2f);            
            SmartDashboard.putNumber("y1", y1f);
            SmartDashboard.putNumber("y2", y2f);

            SmartDashboard.putNumber("theta1a", 180*theta1a/PI);
            SmartDashboard.putNumber("theta1b", 180*theta1b/PI);            
            SmartDashboard.putNumber("theta2a", 180*theta2a/PI);
            SmartDashboard.putNumber("theta2b", 180*theta2b/PI);

            SmartDashboard.putNumber("z1pp", z1pp);
            SmartDashboard.putNumber("z1pn", z1pn);            
            SmartDashboard.putNumber("z1np", z1np);
            SmartDashboard.putNumber("z1nn", z1nn); 

            SmartDashboard.putNumber("z2pp", z2pp);
            SmartDashboard.putNumber("z2pn", z2pn);            
            SmartDashboard.putNumber("z2np", z2np);
            SmartDashboard.putNumber("z2nn", z2nn); 

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
        
        SmartDashboard.putNumber("Tower IK Angle", 180*theta1/PI);
        SmartDashboard.putNumber("Elbow IK Angle", 180*theta2/PI);

        SmartDashboard.putNumber("Target X", targetX);
        SmartDashboard.putNumber("Target Y", targetY);


    }

    
}
    

