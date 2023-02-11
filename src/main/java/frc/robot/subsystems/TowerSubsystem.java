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
    private static AnalogPotentiometer m_PotentiometerWrist = new AnalogPotentiometer(new AnalogInput(2));


    private static CANSparkMax mTower = new CANSparkMax(13, MotorType.kBrushless);
    private static CANSparkMax mElbow = new CANSparkMax(12, MotorType.kBrushless);
    private static TalonSRX m_Wrist = new TalonSRX(16);
    private static XboxController m_Controller = new XboxController(0);
    private static Joystick m_Joystick = new Joystick(1);
    private RelativeEncoder m_encoderTower = mTower.getEncoder();
    private RelativeEncoder m_encoderElbow = mElbow.getEncoder();
    private RelativeEncoder m_encoderWrist;
    private double m_EncoderTowerZero = 0;
    private double m_EncoderTowerMax = 0;
    private double m_EncoderElbowZero = 0;
    private double m_EncoderElbowMax = 0;
    
    private double m_stringTower = 0.0;
    private double m_stringElbow = 0.0;

    private double mTowerSpeed = 0;
    private double mElbowSpeed = 0;
    private double mWristSpeed = .4;

    private boolean IKMode = false;

    private double estX = 0.0;
    private double estY = 0.0;
    private double targetX = 0.82;
    private double targetY = 0.58;
    private double theta1 = 0.0;
    private double theta2 = 0.0;
    private double encoderTowerTarget = m_encoderTower.getPosition();
    private double encoderElbowTarget = m_encoderElbow.getPosition(); 
     
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
        
        boolean isElbowOnTop = !m_ElbowUpProximity.get();
        boolean isElbowOnBottom = !m_ElbowDownProximity.get();
        SmartDashboard.putBoolean("isElbowOnTop", isElbowOnTop);
        SmartDashboard.putBoolean("isElbowOnBottom", isElbowOnBottom);      

        //Built in encoder values 
        // m_encoderElbow = mElbow.getEncoder();
        // m_encoderTower = mTower.getEncoder();

        double encoderElbow = m_encoderElbow.getPosition();
        double encoderTower = m_encoderTower.getPosition();

        if (m_Joystick.getRawButtonPressed(4)){
            IKMode = !IKMode;
        }
        //Moving elbow

        //if (false) {
        if (IKMode == false){
            if (Inputs.xAxisJoystick <= -0.1 || Inputs.xAxisJoystick >= 0.1){
                mElbowSpeed = m_Joystick.getX() * -0.5;
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
        // else // for later, when this actually works
        {
        
            /* String pot readings from new configuration (elbow motor now on "bicep": 2023-02-06) */
            // "Origin" string pot readings
            // Tower: 0.076
            // Elbow 0.805

            // All vertical string pot readings
            // Tower: 0.151
            // Elbow: 0.695

            // Tower vertical, elbow backwards
            // Elbow 0.76
            
            // Tower vertical, elbow forwards
            // Elbow: 0.623

            // All pointing forward (jousting)
            // Tower: 0.10
            // Elbow: 0.695

            // All pointing backward (jousting)
            // Tower: 0.203
            
            /* // Check encoders if we've reached destination, if so, stop.
            if ( Math.abs(encoderElbow - encoderElbowTarget) < 1.0 ) {
                mElbowSpeed = 0.0;
            }

            if ( Math.abs(encoderTower - encoderTowerTarget) < 5.0 ) {
                mTowerSpeed = 0.0;
            } */
   
            /*****************************************************/
            /* IK is still not working, but we're very close!    */
            /* We'll finish it on Saturday (Feb. 11)!  - Dr. Kim */
            /*****************************************************/

            double z1 = 0.4826; //meters
            double z2 = 0.3683; //meters
            double y0 = 0.69; // tower height in meters
            double x0 = 0.0;
            double PI = 3.1415926535;

            double tower_rad = PI*(m_stringPotentiometerTower.get() - 0.099595)/0.11;   // These are from new readings 2023-02-06
            //double tower_rad = PI*(m_encoderTower.getPosition() - m_EncoderTowerZero)/towerEncoderMagicConstant;   // These are from new readings 2023-02-06
            
            double elbow_rad = PI*(m_stringPotentiometerElbow.get() - 0.695)/0.137 + tower_rad;
            //double elbow_rad = PI*(m_encoderElbow.getPosition() - m_EncoderElbowZero)/elbowEncoderMagicConstant;   // These are from new readings 2023-02-06
            
            double tow_y = Math.sin(tower_rad) * z1;
            double tow_x = Math.cos(tower_rad) * z1;
            double elb_y = Math.sin(elbow_rad) * z2;
            double elb_x = Math.cos(elbow_rad) * z2;

            estX = tow_x + elb_x + x0;
            estY = tow_y + elb_y + y0;
            double estTheta1 = Math.atan2(tow_y, tow_x);
            double estTheta2 = Math.atan2(elb_y, elb_x);

            //targetX = estX;
            //targetY = estY;
            
            /* 
            if (Math.abs(Inputs.xAxisJoystick) >= 0.1) { // Dead stick zone
                targetX += m_Joystick.getX() * 0.1; // Very small changes, for now
                if (targetX < 0.1) { // Hard limit for now to keep things forward (positive X)
                    targetX = 0.1;
                }
                //  mTowerSpeed = 0.05; // Very slow speeds for safety
                //  mElbowSpeed = 0.05;
            }
            if (Math.abs(Inputs.yAxisJoystick) >= 0.1) { // Dead stick zone
                targetY += m_Joystick.getY() * 0.1; // Very small changes, for now
                if (targetY > 1.0) {
                    targetY = 1.0;
                }
                //  mTowerSpeed = 0.05;
                //  mElbowSpeed = 0.05;
            } */

            SmartDashboard.putNumber("Tower X", tow_x);
            SmartDashboard.putNumber("Tower Y", tow_y);   
            SmartDashboard.putNumber("Tower angle", 180*(tower_rad)/PI );          
            SmartDashboard.putNumber("Elbow X", elb_x);
            SmartDashboard.putNumber("Elbow Y", elb_y);
            SmartDashboard.putNumber("Elbow angle", 180*(elbow_rad)/PI );            
            SmartDashboard.putNumber("Est X", estX);
            SmartDashboard.putNumber("Est Y", estY);

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

            double x1a = 0.0, x1b = 0.0, x1=0.0;
            double y1a = 0.0, y1b = 0.0, y1=0.0;
            double x2a = 0.0, x2b = 0.0, x2=0.0;
            double y2a = 0.0, y2b = 0.0, y2=0.0;

            // Trying to determine which options are "real" (valid) solutions, based on z1 and z2
            // Set those to 'a' and 'b' (e.g., x1a, y1a, x1b, y1b, etc.)
            if (Math.abs(z1pp - z1) < 0.01 && Math.abs(z2pp - z2) < 0.01) {
                x1a = x1pp; y1a = y1pp;
                x2a = x2pp; y2a = y2pp;
            }
            else {
                x1a = x1pn; y1a = y1pn;
                x2a = x2pn; y2a = y2pn;
            }

            if (Math.abs(z1pn - z1) < 0.01 && Math.abs(z2pn - z2) < 0.01) {
                x1b = x1np; y1b = y1np;
                x2b = x2np; y2b = y2np;
            }
            else {
                x1b = x1nn; y1b = y1nn;
                x2b = x2nn; y2b = y2nn;
            }

            // Find which of the two valid solutions is closest to our current arm position
            double a_dist = Math.abs(x1a-tow_x) + Math.abs(y1a-tow_y) + Math.abs(x2a-tow_x) + Math.abs(y2a-tow_y);
            double b_dist = Math.abs(x1b-tow_x) + Math.abs(y1b-tow_y) + Math.abs(x2b-tow_x) + Math.abs(y2b-tow_y);
        
            if (a_dist < b_dist) {
                x1 = x1a; y1 = y1a; // Option 'a' is best
                x2 = x2a; y2 = y2a;
            }
            else {
                x1 = x1b; y1 = y1b; // Option 'b' is best
                x2 = x2b; y2 = y2b;
            }

            // targetX = x1 + x2 + x0;
            // targetY = y1 + y2 + y0;

            theta1 = Math.atan2(y1, x1); 
            theta2 = Math.atan2(y2, x2);
            double theta1_diff = 0;
            double theta2_diff = 0;

            theta1_diff = theta1 - estTheta1;
            theta2_diff = theta2 - estTheta2;
            
            encoderTowerTarget = encoderTower - (400 * theta1_diff / PI);
            //encoderTowerTarget = encoderTower - (encoderTower180Ticks * theta1_diff / PI);
            encoderElbowTarget = encoderElbow + (48 * theta2_diff / PI);
            //encoderTowerTarget = encoderTower - (encoderElbow180Ticks * theta2_diff / PI);

            SmartDashboard.putNumber("Tower angle diff", theta1_diff);
            SmartDashboard.putNumber("Elbow angle diff", theta2_diff);            
          
            SmartDashboard.putNumber("Tower Enc Target", encoderTowerTarget);
            SmartDashboard.putNumber("Elbow Enc Target", encoderElbowTarget);            
          
            SmartDashboard.putNumber("a", a);
            SmartDashboard.putNumber("b", b);            
            SmartDashboard.putNumber("c", c);
            SmartDashboard.putNumber("x1", x1);
            SmartDashboard.putNumber("x2", x2);            
            SmartDashboard.putNumber("y1", y1);
            SmartDashboard.putNumber("y2", y2);

            SmartDashboard.putNumber("z1pp", z1pp);
            SmartDashboard.putNumber("z1pn", z1pn);            
            SmartDashboard.putNumber("z1np", z1np);
            SmartDashboard.putNumber("z1nn", z1nn); 

            SmartDashboard.putNumber("z2pp", z2pp);
            SmartDashboard.putNumber("z2pn", z2pn);            
            SmartDashboard.putNumber("z2np", z2np);
            SmartDashboard.putNumber("z2nn", z2nn); 
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
            //goToPosition(-55, -12);
            tuckArm();
        }
        if (m_Joystick.getRawButton(5)) {
            goToPosition(0.185, 0.519);
        }

        //Limit Switches to make any last changes to the speed before setting it, must go last!
        if (isArmOnTop && mTowerSpeed > 0) {
            mTowerSpeed = 0;
            m_EncoderTowerZero = m_encoderTower.getPosition();
        } else if (isArmOnBottom && mTowerSpeed < 0) {
            mTowerSpeed = 0;
            m_EncoderTowerMax = m_encoderTower.getPosition();
        }
        if (isElbowOnTop && mElbowSpeed > 0) { 
            mElbowSpeed = 0;
            m_EncoderElbowZero = m_encoderElbow.getPosition();
        } else if (isElbowOnBottom && mElbowSpeed < 0) { 
            mElbowSpeed = 0;
            m_EncoderElbowMax = m_encoderElbow.getPosition();
        }
        
        if (m_PotentiometerWrist.get() < -5 && mWristSpeed < 0){
            mWristSpeed = 0;
        } else if (m_PotentiometerWrist.get() > 5 && mWristSpeed > 0){
            mWristSpeed = 0;
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
        SmartDashboard.putNumber("mTowerSpeed", mTowerSpeed);

        SmartDashboard.putNumber("x-axis controller", Inputs.xAxisJoystick); //Between -1 and 1
        SmartDashboard.putNumber("z-axis controller", Inputs.zAxisJoystick); 

        SmartDashboard.putNumber("String Potentiometer Tower", m_stringPotentiometerTower.get());
        SmartDashboard.putNumber("String Potentiometer Elbow", m_stringPotentiometerElbow.get());
        SmartDashboard.putNumber("Potentiometer Wrist", m_PotentiometerWrist.get());

        SmartDashboard.putNumber("Tower Encoder", encoderTower);
        SmartDashboard.putNumber("Elbow Encoder", encoderElbow);
        
        SmartDashboard.putNumber("Tower IK Angle", 180*theta1/ Math.PI);
        SmartDashboard.putNumber("Elbow IK Angle", 180*theta2/ Math.PI);

        SmartDashboard.putNumber("Target X", targetX);
        SmartDashboard.putNumber("Target Y", targetY);


    }

    
}
    

