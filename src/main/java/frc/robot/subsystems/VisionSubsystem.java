package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Inputs;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.DriveSubsystem;


public class VisionSubsystem extends SubsystemBase{
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    ProfiledPIDController visionAdjustPID = new ProfiledPIDController(10, 0, 0, new TrapezoidProfile.Constraints(1, 1));

    final double xDeadZone = 0.5;



    //read values periodically

    public VisionSubsystem () {
        visionAdjustPID.reset(0);
    }
    

    @Override
    public void periodic() {
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        if (Inputs.m_driverXbox.getLeftStickButtonPressed()){
            visionAdjust(x);
        }
        
    }

    public void visionAdjust(double x) {
         //This will need to be replaced with an inputs button.
        Inputs.driverStrafe = visionAdjustPID.calculate(x, 0);

        if (x > x - xDeadZone && x < x  + xDeadZone ) {
            Inputs.driverPower = -0.1;
        }

    }
}

    


