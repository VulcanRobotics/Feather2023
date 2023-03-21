package frc.robot;
import java.lang.Math;
import frc.robot.Constants;;

public class MathUtil {

    public static double degreesToFalcon(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 2048.0));
    }

    public static double falconToDegrees(double ticks, double gearRatio) {
        return ticks * (360.0 / (gearRatio * 2048.0));
    }

    public static double falconToMeters(double ticks, double wheelCircumference, double gearRatio) {
        return ticks * (wheelCircumference / (gearRatio * 2048.0));
    }

    public static double metersToFalcon(double meters, double wheelCircumference, double gearRatio) {
        return meters / (wheelCircumference / (gearRatio * 2048.0));
    }
    
    
}
