package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.CommandBase; 
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Inputs;
//import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Compressor;

//import frc.robot.Constants;
//import frc.robot.Inputs;
//import frc.robot.LinearServo;

//import java.util.Map;

//import javax.lang.model.util.ElementScanner6;

//import edu.wpi.first.wpilibj.AnalogInput;
//import edu.wpi.first.wpilibj.AnalogPotentiometer;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
//import frc.robot.Constants.OIConstants;

//import edu.wpi.first.wpilibj.XboxController;

public class PneumaticSubsystem extends SubsystemBase {

  private static Solenoid m_clawSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 1);
  private static Solenoid m_intakePullUpSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 2);
  private static Solenoid m_intakePinchSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 3);

  private static Compressor m_compressor = new Compressor(1, PneumaticsModuleType.REVPH); 

  //Solenoid m_solenoid = m_ph.makeSolenoid(0);
  
  private static boolean clawClosed = false;
  private static boolean intakeDeployed = false;
  private static boolean pinchClosed = false;

  public void Pneumatics() {
        m_compressor.enableDigital();
  }

  public void peroidic(){                   // called every robot cycle to ensure state is active
      m_clawSolenoid.set(clawClosed);
      m_intakePullUpSolenoid.set(pinchClosed);
      m_intakePinchSolenoid.set(intakeDeployed);
  }

  public static void toggleIntakeState(){   // sometimes we want to toggle this 
    intakeDeployed = !intakeDeployed;
    m_intakePullUpSolenoid.set(intakeDeployed);
  }

  public static void setIntakeState(boolean state){  // other times like auton we want to force the state
      intakeDeployed = state;
      m_intakePullUpSolenoid.set(state);             // not needed as the period will enforce, maybe we remove
  }

  public static void togglePinchState(){
    pinchClosed = !pinchClosed;
    m_intakePinchSolenoid.set(pinchClosed);
  }

  public static void setPinchState(boolean state){    // force it to this state
    pinchClosed = state;
    m_intakePinchSolenoid.set(state);
  }

  public static void toggleClawState(){
    clawClosed = !clawClosed;
    m_clawSolenoid.set(clawClosed);
  }

  public static void setClawState(boolean state){
    clawClosed = state;
    m_clawSolenoid.set(state);
  }


}