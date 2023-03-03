package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.concurrent.CancellationException;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Inputs;
import frc.robot.Constants;

public class LightingSubsystem extends SubsystemBase{
    private final Spark m_light1 = new Spark(9);

    public double LEDColor = 0.93;
    public double DefaultLEDColor = 0.93;

    @Override
    public void periodic() {


        

        if (Inputs.yellowLED) {
            LEDColor = 0.67;
            Inputs.yellowLED = false;
        } else if (Inputs.purpleLED) {
            LEDColor = 0.91;
            Inputs.purpleLED = false;
        } else if (Inputs.redLED) {
            LEDColor = -0.25;
            Inputs.redLED = false;
        } else if (Inputs.greenLED) {
            LEDColor = -0.53;
            Inputs.greenLED = false;
        } else {
            if (Constants.MatchSettings.kAllianceColor == Constants.MatchSettings.kBlueAlliance) {
                DefaultLEDColor = -0.51;
            } else if (Constants.MatchSettings.kAllianceColor == Constants.MatchSettings.kRedAlliance) {
                DefaultLEDColor = -0.49;
            } else {
                DefaultLEDColor = 0.93;
            }

            LEDColor = DefaultLEDColor;
        }


        m_light1.set(LEDColor);
        SmartDashboard.putBoolean("YELLOW", Inputs.yellowLED);
        SmartDashboard.putBoolean("PURPLE", Inputs.purpleLED);
        SmartDashboard.putNumber("LED COLOR", LEDColor);

    }

    

}
