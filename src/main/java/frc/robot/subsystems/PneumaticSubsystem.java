package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class PneumaticSubsystem extends SubsystemBase {


  //Stating the solenoids
  private static Solenoid m_clawSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 1);
  private static Solenoid m_intakePullUpSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 2);
  private static Solenoid m_intakePinchSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 3);

  //Stating the compressor
  private static Compressor m_compressor = new Compressor(1, PneumaticsModuleType.REVPH); 

  
  public static boolean clawClosed = false;
  public static boolean intakeDeployed = false;
  public static boolean pinchClosed = false;


  //starts by enabling the compressor (automatically stops when full I like that)
  public void Pneumatics() {
        m_compressor.enableDigital();
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