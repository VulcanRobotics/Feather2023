package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Inputs;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

//import javax.swing.text.html.AccessibleHTML.TableElementInfo.TableAccessibleContext;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;



public class VisionSubsystem extends SubsystemBase{
    NetworkTable rear = NetworkTableInstance.getDefault().getTable("limelight-rear");
    NetworkTable front = NetworkTableInstance.getDefault().getTable("limelight-front");

    NetworkTableEntry f_tx = front.getEntry("tx");
    NetworkTableEntry f_ty = front.getEntry("ty");
    NetworkTableEntry f_ta = front.getEntry("ta");
    NetworkTableEntry f_tv = front.getEntry("tv");
    
    
    NetworkTableEntry r_tx = rear.getEntry("tx");
    NetworkTableEntry r_ty = rear.getEntry("ty");
    NetworkTableEntry r_ta = rear.getEntry("ta");
    NetworkTableEntry r_tv = rear.getEntry("tv");

    NetworkTableEntry pipeline = rear.getEntry("pipeline");
    

     ProfiledPIDController visionAdjustPID = new ProfiledPIDController(0.02, 0, 0, new TrapezoidProfile.Constraints(1, 1));
     ProfiledPIDController turnAdjustPID = new ProfiledPIDController(0.04, 0, 0, new TrapezoidProfile.Constraints(1, 1));

    final double xDeadZone = 0.5;

    

    //read values periodically

    public VisionSubsystem () {
        visionAdjustPID.reset(0);
        pipeline.setDouble(0);
    }
    

    @Override
    public void periodic() {
        double x = r_tx.getDouble(0.0);
        double y = r_ty.getDouble(0.0);
        double area = r_ta.getDouble(0.0);
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
    
    public double[] visionAdjustX(boolean front) {
        
        if (!front) {
            double dist = r_tx.getDouble(0.0);
            return new double[]{visionAdjustPID.calculate(dist, 0), turnAdjustPID.calculate(DriveSubsystem.m_gyro.getYaw(), 0)};
       } else {
            double dist = f_tx.getDouble(0.0);
            return new double[]{visionAdjustPID.calculate(dist, 0), turnAdjustPID.calculate(dist, 0)};
       }
 
        
    }

    public double targetCube() {
        double rot = f_tx.getDouble(0.0);
        return turnAdjustPID.calculate(rot, 0);
    }


    public boolean areWeCentered(double tolerance) {
        double dist = r_tx.getDouble(0.0);
        double area = r_ta.getDouble(0.0);
        if (Math.abs(dist) < tolerance) {
            return true;
        } else {
            return false;
        }


    }

    public boolean limelightHasTarget(){
        long target = r_tv.getInteger(0);
        if (target == 1){
            return true;
        } else {
            return false;
        }
    }

    public void switchPipeline(int pipelineNumber) {
        pipeline.setDouble(pipelineNumber);
    }

    public long getPipelineNumber() {
        return pipeline.getInteger(0);
    }

}

    


