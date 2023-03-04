package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.concurrent.CancellationException;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Inputs;
import frc.robot.Constants;

public class LightingSubsystem extends SubsystemBase{
    private final Spark m_light1 = new Spark(9);

    public double LEDColor = 0.93; //solid white
    public double DefaultLEDColor = 0.93; //solid white

    @Override
    public void periodic() {


        

        if (Inputs.yellowLED) {
            LEDColor = 0.67; //solid gold
            Inputs.yellowLED = false;
        } else if (Inputs.purpleLED) {
            LEDColor = 0.91; //solid violet
            Inputs.purpleLED = false;
        } else if (Inputs.redLED) {
            LEDColor = -0.47; //Twinkles, forest palette
            Inputs.redLED = false;
        } else if (Inputs.greenLED) {
            LEDColor = -0.53; //Twinkles, party palette
            Inputs.greenLED = false;                  
        } else {
            if (Constants.MatchSettings.kAllianceColor == Constants.MatchSettings.kBlueAlliance) {
                DefaultLEDColor = -0.51; //Twinkles, ocean palette
            } else if (Constants.MatchSettings.kAllianceColor == Constants.MatchSettings.kRedAlliance) {
                DefaultLEDColor = -0.49; //Twinkles, lava palette
            } else {
                DefaultLEDColor = 0.93; //solid white
            }

            LEDColor = DefaultLEDColor;
        }


        m_light1.set(LEDColor);
        SmartDashboard.putBoolean("YELLOW", Inputs.yellowLED);
        SmartDashboard.putBoolean("PURPLE", Inputs.purpleLED);
        SmartDashboard.putNumber("LED COLOR", LEDColor);

    }

    

}
