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
    NetworkTableEntry tv = table.getEntry("tv");

 

     ProfiledPIDController visionAdjustPID = new ProfiledPIDController(0.025, 0, 0, new TrapezoidProfile.Constraints(1, 1));
     ProfiledPIDController turnAdjustPID = new ProfiledPIDController(0.015, 0, 0, new TrapezoidProfile.Constraints(1, 1));

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
        /* 
        if (Inputs.m_driverXbox.getXButtonPressed()) {
            Inputs.driverStrafe = visionAdjustX();
            SmartDashboard.putNumber("Vision adjust PID OUT", visionAdjustPID.calculate(x, 0));
        }
        */
        
    }

    public double[] visionAdjustX() {
        double dist = tx.getDouble(0.0);
       //if (Math.abs(dist) > 6) {
            return new double[]{visionAdjustPID.calculate(dist, 0), turnAdjustPID.calculate(DriveSubsystem.m_gyro.getYaw(), 0)};
 
        
    }

    public boolean areWeCentered() {
        double dist = tx.getDouble(0.0);
        double area = ta.getDouble(0.0);
        if (Math.abs(dist) < 3) {
            return true;
        } else {
            return false;
        }


    }

    public boolean limelightHasTarget(){
        long target = tv.getInteger(0);
        if (target == 1){
            return true;
        } else {
            return false;
        }
    }
}

    


