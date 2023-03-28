package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Inputs;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.function.LongToDoubleFunction;

//import javax.swing.text.html.AccessibleHTML.TableElementInfo.TableAccessibleContext;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;



public class VisionSubsystem extends SubsystemBase{


    //stating the tables that the limelights write on
    NetworkTable rear = NetworkTableInstance.getDefault().getTable("limelight-rear");
    NetworkTable front = NetworkTableInstance.getDefault().getTable("limelight-front");


    //stating the data given from the tables that the limelightes write on
    NetworkTableEntry f_tx = front.getEntry("tx");
    NetworkTableEntry f_ty = front.getEntry("ty");
    NetworkTableEntry f_ta = front.getEntry("ta");
    NetworkTableEntry f_tv = front.getEntry("tv");
    
    //Same thing, but for the front camera (signified by the "r_" representing rear)
    NetworkTableEntry r_tx = rear.getEntry("tx");
    NetworkTableEntry r_ty = rear.getEntry("ty");
    NetworkTableEntry r_ta = rear.getEntry("ta");
    NetworkTableEntry r_tv = rear.getEntry("tv");

    //stating the pipeline number on the rear limelight, we switch between two pipeline during matchplay (one for retroreflective tape, the other for april tags)
    NetworkTableEntry pipeline = rear.getEntry("pipeline");
    
    //stating PID controllers for limelights
     ProfiledPIDController visionAdjustPID = new ProfiledPIDController(0.02, 0, 0, new TrapezoidProfile.Constraints(1, 1));
     ProfiledPIDController turnAdjustPID = new ProfiledPIDController(0.03, 0, 0, new TrapezoidProfile.Constraints(1, 1));

    final double xDeadZone = 0.5;
    public static double pipelineNumber = 0.0;

    

    //read values periodically

    public VisionSubsystem () {
        visionAdjustPID.reset(0);
        pipeline.setDouble(0);
    }
    

    @Override
    public void periodic() {
        
        pipelineNumber = pipeline.getInteger(0);

        //Just displays the variables of the rear limelight for analysis/viewing
        double x = r_tx.getDouble(0.0);
        double y = r_ty.getDouble(0.0);
        double area = r_ta.getDouble(0.0);
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        
    }
    
    //This is the main automatic vision adjust function that uitilizes the two limelights to automatically strafe/turn towards the specified target
    public double[] visionAdjustX(boolean front) {
        
        if (!front) {
            double dist = r_tx.getDouble(0.0);
            return new double[]{visionAdjustPID.calculate(dist, 0), turnAdjustPID.calculate(DriveSubsystem.m_gyro.getYaw(), 0)};
       } else {
            double dist = f_tx.getDouble(0.0);
            return new double[]{visionAdjustPID.calculate(dist, 0), turnAdjustPID.calculate(dist, 0)};
       }
 
        
    }



    //This function helps let the if statement know whether we are on target or not
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

    //This switches the pipeline for the rear limelight
    public void switchPipeline(int pipelineNumber) {
        pipeline.setDouble(pipelineNumber);
    }

    //This gives the current pipeline number for SmartDashboard Reasons

}

    


