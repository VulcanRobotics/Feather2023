package frc.robot.subsystems;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase; 
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Compressor;

import frc.robot.Constants;
import frc.robot.Inputs;
import frc.robot.LinearServo;

import java.util.Map;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants.OIConstants;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;

public class PneumaticSubsystem extends SubsystemBase {

  private static Joystick m_Joystick = new Joystick(1);
  private static Solenoid m_clawSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 1);
  private static Solenoid m_intakePullUpSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 2);
  private static Solenoid m_intakePinchSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 3);

  private static Compressor m_compressor = new Compressor(1, PneumaticsModuleType.REVPH); 

  //Solenoid m_solenoid = m_ph.makeSolenoid(0);


  private boolean clawClosed = false;

  public static void toggleIntake(boolean toggle){
      m_intakePullUpSolenoid.set(toggle);
  }

  public static void togglePinch(boolean toggle){
    m_intakePinchSolenoid.set(toggle);
}

  public void Pneumatics() {
        m_compressor.enableDigital();

        if (m_Joystick.getRawButtonPressed(1)){
          clawClosed = !clawClosed;
        }

        if (clawClosed){
          m_clawSolenoid.set(true);
        } else {
          m_clawSolenoid.set(false);
        }
    }
}