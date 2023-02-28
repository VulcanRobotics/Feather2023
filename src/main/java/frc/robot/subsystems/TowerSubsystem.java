package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Inputs;

//import frc.robot.LinearServo;
import frc.robot.subsystems.PneumaticSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;

import javax.swing.text.StyledEditorKit.BoldAction;
import javax.swing.text.html.HTMLDocument.BlockElement;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.Tower.AutonFlags;
import edu.wpi.first.wpilibj.XboxController;

public class TowerSubsystem extends SubsystemBase {

    static DigitalInput m_towerUpProximity = new DigitalInput(2);
    DigitalInput m_towerDownProximity = new DigitalInput(3);
    DigitalInput m_ElbowDownProximity = new DigitalInput(8);
    static DigitalInput m_ElbowUpProximity = new DigitalInput(7);

    private static AnalogPotentiometer m_stringPotentiometerTower = new AnalogPotentiometer(new AnalogInput(0));
    private static AnalogPotentiometer m_stringPotentiometerElbow = new AnalogPotentiometer(new AnalogInput(1));
    private static AnalogPotentiometer m_PotentiometerWrist = new AnalogPotentiometer(new AnalogInput(2));

    //private static CANSparkMax mTower = new CANSparkMax(13, MotorType.kBrushless);
    //private static CANSparkMax mElbow = new CANSparkMax(12, MotorType.kBrushless);
    //private static TalonSRX m_Wrist = new TalonSRX(16);

    private static TalonFX mTower = new TalonFX(13, "roborio");
    private static TalonFX mElbow = new TalonFX(12, "roborio");
    
    private static CANSparkMax m_Wrist = new CANSparkMax(3, MotorType.kBrushless);
    //private static XboxController m_Controller = new XboxController(0);
    //private static Joystick m_Joystick = new Joystick(1);
    //private static Joystick m_BoxController = new Joystick(3);
    //private RelativeEncoder m_towerEncoder;// = mTower.getEncoder();
    //private RelativeEncoder m_elbowEncoder;// = mElbow.getEncoder();
    //private RelativeEncoder m_towerEncoder = mTower.getEncoder();
    //private RelativeEncoder m_elbowEncoder = mElbow.getEncoder();
    private RelativeEncoder m_encoderWrist = m_Wrist.getEncoder();
    private double m_towerEncoderZero = 0;
    private double m_elbowEncoderZero = 0;
    
    private boolean m_useTowerEncoder = false;
    private boolean m_useElbowEncoder = false;

    // These should be constants
    private static double towerEncoder180Ticks = 400.0;
    private static double elbowEncoder180Ticks = 49.65;
    private static double m_towerEncoderMax = -383.340546 - 135.450546; // -518.791092
    private static double m_elbowEncoderMax = 73.00 - (-8.50); // 81.5
    
    private static double m_stringTower = 0.0;
    private static double m_stringElbow = 0.0;
    private double m_wristEncoder = 0.0;

    private static double mTowerSpeed = 0;
    private static double mElbowSpeed = 0;
    private double mWristSpeed = .4;

    private boolean IKMode = false;
    private boolean wristFlipped = true;
    private boolean wristLock = true;

    private double estX = 0.0;
    private double estY = 0.0;
    private double targetX = 0.82;
    private double targetY = 0.58;
    private double theta1 = 0.0;
    private double theta2 = 0.0;
    private double towerEncoderTarget = mTower.getSelectedSensorPosition();
    private double elbowEncoderTarget = mElbow.getSelectedSensorPosition(); 

    private double initialWristEncoder = m_encoderWrist.getPosition();
    private final double wristEncoder180 = 47.642;

    private static boolean elbowMoveTo(double Evalue){
        boolean done = false;

        if (m_stringElbow >= Evalue + .01) {
            mElbowSpeed = -0.3;
        } else if (m_stringElbow <= Evalue - .01) {
            mElbowSpeed = 0.3;
        } else if (m_stringElbow >= Evalue - 0.01) {
            mElbowSpeed = 0;
        } else if (m_stringElbow <= Evalue + 0.01) {
            mElbowSpeed = 0;
        }

        if (m_stringElbow <= Evalue + 0.01 && m_stringElbow >= Evalue - 0.01){
            done = true;
        }

        return done;
    }

    private static boolean towerMoveTo(double Tvalue){
    boolean done = false;

        if (m_stringTower >= Tvalue + .05) {
            mTowerSpeed = 1;
        } else if (m_stringTower <= Tvalue - .05) {
            mTowerSpeed = -1;
        } else if (m_stringTower >= Tvalue + .01) {
            mTowerSpeed = 0.7;
        } else if (m_stringTower <= Tvalue - .01) {
            mTowerSpeed = -0.7;
        } else if (m_stringTower >= Tvalue - 0.01) {
            mTowerSpeed = 0;
        } else if (m_stringTower <= Tvalue + 0.01) { 
            mTowerSpeed = 0;
        }

        if (m_stringTower <= Tvalue + 0.01 && m_stringTower >= Tvalue - 0.01){
            done = true;
        }

        return done;
    }
     
    private static boolean goToPosition(double Tvalue, double Evalue, boolean towerPriority, boolean flipArm) {
        m_stringTower = m_stringPotentiometerTower.get();
        m_stringElbow = m_stringPotentiometerElbow.get();

        boolean elbowDone = false;
        boolean towerDone = false;

        if (towerPriority){
            if (towerDone = towerMoveTo(Tvalue)){
                elbowDone = elbowMoveTo(Evalue);
            }
        }else if (flipArm){
            if (m_stringTower > 0.38){
                elbowDone = elbowMoveTo(Evalue);

                towerDone = towerMoveTo(Tvalue);
            } else{
                towerDone = towerMoveTo(Tvalue);
            }
        } else{
            towerDone = towerMoveTo(Tvalue);
    
            elbowDone = elbowMoveTo(Evalue);
        }

        if (towerDone && elbowDone){
            return true;
        } else{
            return false;
        }

    }

    public void flipWrist() { //DOES NOT WORK
        if ((wristEncoder180 - m_encoderWrist.getPosition() > initialWristEncoder - m_encoderWrist.getPosition())) {
            m_Wrist.set(0.1);
        } else {
            m_Wrist.set(-0.1);
        }
    }

    public static boolean highPlace(){
        return goToPosition(.511, .796, false, true);
    }
    public boolean midPlace(){
        return goToPosition(.336, .792 , false, false);
    }
    public boolean humanPlayerGrab(){
        return goToPosition(.390, .713, false, false);
    }

    public boolean grabFromIntake() {
        return goToPosition(0.322, 0.71, true, false);
    }

    public static void tuckArm(){
        if (!m_towerUpProximity.get() == true){
            mTowerSpeed = 0.85;
        }
        if (m_ElbowUpProximity.get()){
            mElbowSpeed = 0.25;
        }
    }

    private void goToWristPosition(double wristValue){
        SmartDashboard.putNumber("wrist target", wristValue);

        if (m_wristEncoder <  wristValue - 3) {
            mWristSpeed = 0.04;
        } else if (m_wristEncoder > wristValue + 3) {
            mWristSpeed = -0.04;
        } else{
            mWristSpeed = 0;
        }
    }

    public void periodic(){

        boolean isArmOnTop = m_towerUpProximity.get();
        boolean isArmOnBottom = m_towerDownProximity.get();
        
        boolean isElbowOnTop = !m_ElbowUpProximity.get();
        boolean isElbowOnBottom = !m_ElbowDownProximity.get();
        //SmartDashboard.putBoolean("isElbowOnTop", isElbowOnTop);
        //SmartDashboard.putBoolean("isElbowOnBottom", isElbowOnBottom);      

        //Built in encoder values 
        // m_elbowEncoder = mElbow.getEncoder();
        // m_towerEncoder = mTower.getEncoder();
        m_wristEncoder = m_encoderWrist.getPosition();

        double elbowEncoder = mElbow.getSelectedSensorPosition();
        double towerEncoder = mTower.getSelectedSensorPosition();

        if (Inputs.m_operatorControl.getTriggerPressed()){
            PneumaticSubsystem.toggleClawState();
        }

        if (false){
            IKMode = !IKMode;
        }
        //Moving elbow

        //if (false) {
        if (IKMode == false){

            mElbowSpeed = Inputs.applyDeadBand(Inputs.towerElbowPower,.1) * Constants.Tower.kElbowPCTPower;
            //if (Inputs.xAxisJoystick <= -0.1 || Inputs.xAxisJoystick >= 0.1){
            //    mElbowSpeed = m_Joystick.getX() * -0.8;
            //} else {
            //    mElbowSpeed = 0;
            //}
            
            mTowerSpeed = -Inputs.applyDeadBand(Inputs.towerShoulderPower, .1) * Constants.Tower.kShoulderPCTPower ;
            //Moving shoulder
            //shaun - changed 1/25/23 - inverted y and added bottom potentiometer limit. The variable tower bottom is at the top of file.
            //if (Math.abs(Inputs.yAxisJoystick) >= 0.1) { //Checking if the arm is above stringpot limit, also applying deadband
            //    mTowerSpeed = m_Joystick.getY() * -1; //Inverted y-axis controls, was -2
                
            //} else {
            //    mTowerSpeed = 0.0;
            //}
        }
         else // for later, when this actually works
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
            
            // Check encoders if we've reached destination, if so, stop.
            /*if ( Math.abs(elbowEncoder - elbowEncoderTarget) < 1.0 ) {
                mElbowSpeed = 0.0;
            }

            if ( Math.abs(towerEncoder - towerEncoderTarget) < 5.0 ) {
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
            double tower_rad = 0.0;
            double elbow_rad = 0.0;

            if (m_useTowerEncoder == true) {
                tower_rad = PI*(mTower.getSelectedSensorPosition() - m_towerEncoderZero)/-towerEncoder180Ticks;   // These are from new readings 2023-02-06
            }
            else {
                // Use string potentiometer
                // tower_rad = PI*(m_stringPotentiometerTower.get() - 0.099595)/0.11;   // These are from new readings 2023-02-06: horizontal 0 degrees
                tower_rad = PI*(m_stringPotentiometerTower.get() - 0.074) / 0.11;   // This is the tower at zero limit
            }

            if (m_useElbowEncoder == true) {
                elbow_rad = PI*(mElbow.getSelectedSensorPosition() - m_elbowEncoderZero)/elbowEncoder180Ticks;   // These are from new readings 2023-02-06
            }
            else {
                // Use string pot
                // elbow_rad = PI*(m_stringPotentiometerElbow.get() - 0.695)/0.137 + tower_rad; // Horizontal
                elbow_rad = PI*(m_stringPotentiometerElbow.get() - 0.575)/0.137; // At the proximity limit
            }            
            
            // Zero position is -43.4 degrees -->> -0.7574729 rad
            double to_horizontal = -0.7574729;

            // Zero of elbow to arm is -129.2 degrees -->> -0.2549654 rad
            double to_arm = -2.2549654;

            // We want these values in "normal" (horizontal-vertical) coordinates
            double tower_rad_horizontal = tower_rad + to_horizontal;
            double elbow_rad_horizontal = elbow_rad + to_arm + tower_rad_horizontal;

            double tow_y = Math.sin(tower_rad + to_horizontal) * z1;
            double tow_x = Math.cos(tower_rad + to_horizontal) * z1;
            double elb_y = Math.sin(elbow_rad + to_arm + to_horizontal) * z2;
            double elb_x = Math.cos(elbow_rad + to_arm + to_horizontal) * z2;
            estX = tow_x + elb_x + x0;
            estY = tow_y + elb_y + y0;

            //double estTheta1 = Math.atan2(tow_y, tow_x) ;
            //double estTheta2 = Math.atan2(elb_y, elb_x);

            //targetX = estX;
            //targetY = estY;
                    
            if (Inputs.m_operatorControl.getRawButton(7) == true)  { 
                targetX -= 0.1;
                // mTowerSpeed = 0.05; // Very slow speeds for safety
                // mElbowSpeed = 0.05;
            }
            if (Inputs.m_operatorControl.getRawButton(8) == true)  { 
                targetX += 0.1;
                // mTowerSpeed = 0.05; // Very slow speeds for safety
                // mElbowSpeed = 0.05;
            }
            if (Inputs.m_operatorControl.getRawButton(9) == true) {
                targetY -= 0.1;
                // mTowerSpeed = 0.05;
                // mElbowSpeed = 0.05;
            }
            if (Inputs.m_operatorControl.getRawButton(10) == true) {
                targetY += 0.1;
                // mTowerSpeed = 0.05;
                // mElbowSpeed = 0.05;
            }

            if( Constants.DashboardSwitches.TowerDisplayOn){
                SmartDashboard.putNumber("Tower X", tow_x);
                SmartDashboard.putNumber("Tower Y", tow_y);   
                SmartDashboard.putNumber("Tower angle", 180*(tower_rad)/PI );          
                SmartDashboard.putNumber("Elbow X", elb_x);
                SmartDashboard.putNumber("Elbow Y", elb_y);
                SmartDashboard.putNumber("Elbow angle", 180*(elbow_rad)/PI );            
                SmartDashboard.putNumber("Est X", estX);
                SmartDashboard.putNumber("Est Y", estY);
                SmartDashboard.putNumber("Elbow ang to horiz", 180*elbow_rad_horizontal/Math.PI);
                SmartDashboard.putNumber("Tower ang to horiz", 180*tower_rad_horizontal/Math.PI);
                SmartDashboard.putBoolean("isElbowOnTop", isElbowOnTop);
                SmartDashboard.putBoolean("isElbowOnBottom", isElbowOnBottom);      
            }

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

            // Target angles (in reference to X-Y horizontal-vertical)
            theta1 = Math.atan2(y1, x1); 
            theta2 = Math.atan2(y2, x2);

            if (theta1 < 0) { theta1 += 2*Math.PI; }
            if (theta2 < 0) { theta2 += 2*Math.PI; }

            theta1 += to_horizontal;
            theta2 += to_arm - theta1; // theta1 has already been converted

            double theta1_diff = 0;
            double theta2_diff = 0;

            // Compare angles in "natural" rotation (zero at one limit)
            theta1_diff = theta1 - tower_rad; // estTheta1;
            theta2_diff = theta2 - elbow_rad; // estTheta2;
            
            //towerEncoderTarget = towerEncoder - (400 * theta1_diff / PI);
            towerEncoderTarget = towerEncoder + (towerEncoder180Ticks * theta1_diff / PI);
            //elbowEncoderTarget = elbowEncoder + (48 * theta2_diff / PI);
            elbowEncoderTarget = elbowEncoder - (elbowEncoder180Ticks * theta2_diff / PI); // Is elbow going backwards?

            if( Constants.DashboardSwitches.TowerDisplayOn){
                SmartDashboard.putNumber("Tower angle diff", theta1_diff);
                SmartDashboard.putNumber("Elbow angle diff", theta2_diff);            
            
                SmartDashboard.putNumber("Tower Enc Target", towerEncoderTarget);
                SmartDashboard.putNumber("Elbow Enc Target", elbowEncoderTarget);            
            
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
        }


        //For wrist movement, rotate joystick
        mWristSpeed = 0.0;
        if( Math.abs(Inputs.towerWristSpeed) > .6){
            if( Inputs.towerWristSpeed > 0.0)
                mWristSpeed = Constants.Tower.kWristMaxPower;
            else if( Inputs.towerWristSpeed < 0.0)
                mWristSpeed = -Constants.Tower.kWristMaxPower;
        }

		// fwl - this code was still in cybersonics code
        //if (Inputs.zAxisJoystick <= -0.4 || Inputs.zAxisJoystick >= 0.4){
        //    mWristSpeed = 0.2;
        //} else {
        //    mWristSpeed = 0.0;
        //} 

        /*if (Inputs.m_operatorControl.getRawButtonPressed(2)) {
            wristFlipped = !wristFlipped;
        }*/
        
        if (wristLock == true){
	        if (m_encoderWrist.getPosition() < initialWristEncoder - wristEncoder180 && mWristSpeed < 0){
	            mWristSpeed = 0;
	        }else if (m_encoderWrist.getPosition() > initialWristEncoder && mWristSpeed > 0){
	            mWristSpeed = 0;
			}
        }

        /*if (Math.abs(Inputs.zAxisJoystick) < 0.4) {
            if(wristFlipped){
                goToWristPosition(initialWristEncoder);
            }else{
                goToWristPosition(initialWristEncoder - wristEncoder180);
            }
        }*/

        //Uitilizing the goToPosition function that moves the robot to a certain position based off of the values inputted
        if (Inputs.m_operatorControl.getRawButton(2)) {
            //goToPosition(-55, -12);
            tuckArm();
        }




        switch (Inputs.autonRequestGoTo){
            case IGNORE: 
                break;

            case HIGHPLACE:
                boolean done = highPlace();

                if (done) {
                    PneumaticSubsystem.setClawState(true);
                }
                break;
            
            case ORIGIN:
                tuckArm();
                break;
                
            default:
                break;
        }

   
        if (Inputs.m_operatorControl.getRawButton(3)) {
            humanPlayerGrab();
        }

        if (Inputs.m_operatorControl.getRawButton(4)) {
            midPlace();
        }

        if (Inputs.m_operatorControl.getRawButton(5)) {
            grabFromIntake();
        }

        if (Inputs.m_operatorControl.getRawButton(6)) {
            highPlace();
        }

        if (Inputs.m_operatorControl.getRawButton(8)) {
            wristLock = false;
        }

        if (Inputs.m_operatorControl.getRawButtonReleased(8)){
            wristLock = true;
            initialWristEncoder = m_wristEncoder;
        }
        

        //Limit Switches to make any last changes to the speed before setting it, must go last!
        if (isArmOnTop && mTowerSpeed > 0) {
            mTowerSpeed = 0;
            if (m_useTowerEncoder == false) {
                m_useTowerEncoder = true;
                m_towerEncoderZero = mTower.getSelectedSensorPosition();
            }
        } else if (isArmOnBottom && mTowerSpeed < 0) {
            mTowerSpeed = 0;
            if (m_useTowerEncoder == false) {
                m_useTowerEncoder = true;
                m_towerEncoderZero = mTower.getSelectedSensorPosition() - m_towerEncoderMax;
            }
        }

        if (isElbowOnTop && mElbowSpeed > 0) { 
            mElbowSpeed = 0;
            if (m_useElbowEncoder == false) {
                m_useElbowEncoder = true;
                m_elbowEncoderZero = mElbow.getSelectedSensorPosition();
            }
        } else if (isElbowOnBottom && mElbowSpeed < 0) { 
            mElbowSpeed = 0;
            if (m_useElbowEncoder == false) {
                m_useElbowEncoder = true;
                m_elbowEncoderZero = mElbow.getSelectedSensorPosition() - m_elbowEncoderMax;
            }
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

        //mTower.set(mTowerSpeed);
        //mElbow.set(mElbowSpeed);
        mTower.setNeutralMode(NeutralMode.Brake);
        mElbow.setNeutralMode(NeutralMode.Brake);
        //SmartDashboard.putNumber("mTowerSpeed", mTowerSpeed);
        mTower.set(ControlMode.PercentOutput, mTowerSpeed);
        mElbow.set(ControlMode.PercentOutput, mElbowSpeed);

        //m_Wrist.set(ControlMode.PercentOutput, mWristSpeed);
        m_Wrist.set(mWristSpeed);
         
        if( Constants.DashboardSwitches.TowerDisplayOn){
            SmartDashboard.putNumber("mElbow", m_stringPotentiometerElbow.get());
            SmartDashboard.putNumber("mTowerSpeed", mTowerSpeed);
            SmartDashboard.putBoolean("upperProximity", isElbowOnTop);
            SmartDashboard.putBoolean("lowerProximity", isElbowOnBottom);
            SmartDashboard.putBoolean("Elbow down Prox", m_ElbowDownProximity.get());
            SmartDashboard.putBoolean("ikmode", IKMode);
            SmartDashboard.putNumber("mElbowSpeed", mElbowSpeed);
            SmartDashboard.putNumber("mWristSpeed", mWristSpeed);
        
            SmartDashboard.putNumber("initial wrist encoder", initialWristEncoder);

            SmartDashboard.putNumber("String Potentiometer Tower", m_stringPotentiometerTower.get());
            SmartDashboard.putNumber("String Potentiometer Elbow", m_stringPotentiometerElbow.get());
            SmartDashboard.putNumber("Wrist Encoder", m_wristEncoder);

        	SmartDashboard.putNumber("Tower Encoder", towerEncoder);
        	SmartDashboard.putNumber("Elbow Encoder", elbowEncoder);

        	SmartDashboard.putNumber("Tower Encoder", mTower.getSelectedSensorPosition());
        	SmartDashboard.putNumber("Elbow Encoder", mElbow.getSelectedSensorPosition());
        
        	SmartDashboard.putNumber("Tower IK Angle", 180*theta1/ Math.PI);
        	SmartDashboard.putNumber("Elbow IK Angle", 180*theta2/ Math.PI);

        	SmartDashboard.putNumber("Target X", targetX);
        	SmartDashboard.putNumber("Target Y", targetY);

        }



    }

    
}
    

