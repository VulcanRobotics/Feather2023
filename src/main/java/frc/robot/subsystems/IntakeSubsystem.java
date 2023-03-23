package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.PneumaticSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.swing.plaf.synth.SynthDesktopIconUI;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Inputs;
import frc.robot.Robot;


public class IntakeSubsystem extends SubsystemBase {
    public static CANSparkMax m_rightPincerMotor = new CANSparkMax(17, MotorType.kBrushless);
    public static CANSparkMax m_leftPincerMotor = new CANSparkMax(18, MotorType.kBrushless);

    public static final DigitalInput m_intakePhotogate = new DigitalInput(9);

    private XboxController m_driverXbox = Inputs.m_driverXbox;

    public static double intakeSpeed = 0.0;
    public static double leftPincerSpeed = 0.0;

    private double startTime = System.currentTimeMillis();
    private double elapsedtime = 0.0;
    private boolean startClock = true;
    private boolean haveCube = false;
    private boolean firstPass = true;

    public boolean getIntakePhotogate() {
        return m_intakePhotogate.get();
    }

// This function assists in keeping the wheels spin for a short period of time after letting go of the button. Makes sure that the cube is in place when raising the intake
    public void keepSpinning(){
        
        if (startClock == true){
            startTime = System.currentTimeMillis();
            startClock = false;
            elapsedtime = 0.0;
        }
        else {
            elapsedtime = System.currentTimeMillis() - startTime;
        }
    
        if (elapsedtime < 500) { //currently 0.5 seconds
            intakeSpeed = 0.5;
        } else {
            intakeSpeed = 0.0;
        }
    }

    public void periodic(){

        SmartDashboard.putBoolean("PHOTOGATE", getIntakePhotogate());

        
        //Detects whether the photogate senses something or not, changing the variable depending on the value
        if (getIntakePhotogate()) {
            haveCube = true;
        } else {
            haveCube = false;
        }

        //if the left trigger is pushed, the intake goes down and the clock (for the function) turns on if the cube is sensed
        if (m_driverXbox.getLeftTriggerAxis() > 0.1){
            PneumaticSubsystem.setIntakeState(true);
            intakeSpeed = 0.5;
            if (haveCube){
                if (!startClock && firstPass){
                    startClock = true;
                    firstPass = false;
                }
                
            } else  {
                intakeSpeed = 0.5;
            }
        } else {
            PneumaticSubsystem.setIntakeState(false);
        }
        //this just reverses the motors, spitting the cube out
        if (m_driverXbox.getRightTriggerAxis() > 0.1){
            intakeSpeed = -0.5;
            haveCube = false;
        }
        //these are cases used for autonomous
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
        
        

 
        //This finally goes over the conditions and gives the motor power dependent on which is satified
        if (haveCube) { //If you have the cube, stop the motors
            m_leftPincerMotor.set(0);
        } else if (m_driverXbox.getLeftTriggerAxis() < 0.1 && m_driverXbox.getRightTriggerAxis() < 0.1){ //if nothing is being pressed, have the motors spin in slowly
            m_leftPincerMotor.set(0.25);
        } else { //if you are pressing a trigger though, set the speed normally
            m_leftPincerMotor.set(intakeSpeed);
        }

        

        SmartDashboard.putBoolean("Intake Deployed", PneumaticSubsystem.intakeDeployed);

    }

    //This is a function used in auton to spin the motors in
    public static void spinMotors(boolean reversed) {
        if (!reversed) {
            m_leftPincerMotor.set(0.5);
        } else {
            m_leftPincerMotor.set(0.5*-1);
        }
    }

}

