package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.concurrent.CancellationException;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Inputs;

public class LightingSubsystem extends SubsystemBase{
    private final Spark m_light1 = new Spark(9);

    public double LEDColor = 0.93;

    @Override
    public void periodic() {


        if (Inputs.yellowLED) {
            LEDColor = 0.67;
            Inputs.yellowLED = false;
        } else if (Inputs.purpleLED) {
            LEDColor = 0.91;
            Inputs.purpleLED = false;
        } else if (Inputs.redLED) {
            LEDColor = -0.39;
            Inputs.redLED = false;
        } else if (Inputs.greenLED) {
            LEDColor = -0.37;
            Inputs.greenLED = false;
        } else {
            LEDColor = 0.93;
        }


        m_light1.set(LEDColor);
        SmartDashboard.putBoolean("YELLOW", Inputs.yellowLED);
        SmartDashboard.putBoolean("PURPLE", Inputs.purpleLED);
        SmartDashboard.putNumber("LED COLOR", LEDColor);

    }

    

}
