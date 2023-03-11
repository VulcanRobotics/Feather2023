// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants;
import frc.robot.Inputs;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Ultrasonic;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class DriveSubsystem extends SubsystemBase {

  
  /******************************************************** 
    Compiler complaining this is dead code. 
  private double getRobotSpecificOffset(int wheel){
    double[] heatherOffsets = new double[]
    { 
      DriveConstants.kHeatherFrontLeftTurnEncoderOffsetDeg, DriveConstants.kHeatherRearLeftTurnEncoderOffsetDeg,
      DriveConstants.kHeatherFrontRightTurnEncoderOffsetDeg, DriveConstants.kHeatherRearRightTurnEncoderOffsetDeg
    };
    
    double[] perseveranceOffsets = new double[]
    {
      DriveConstants.kPerseveranceFrontLeftTurnEncoderOffsetDeg, DriveConstants.kPerseveranceRearLeftTurnEncoderOffsetDeg,
      DriveConstants.kPerseveranceFrontRightTurnEncoderOffsetDeg, DriveConstants.kPerseveranceRearRightTurnEncoderOffsetDeg
    };
    
    if (Constants.whichRobot == "Heather") 
      return heatherOffsets[wheel];
    else 
      return perseveranceOffsets[wheel];

  }
  ****************************************************/

  // Robot swerve modules
  public static final SwerveModule m_frontLeft =
      new SwerveModule(
          DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontLeftTurningMotorPort,
          DriveConstants.kFrontLeftTurningEncoderPorts,
          DriveConstants.kFrontLeftDriveEncoderReversed,
          DriveConstants.kFrontLeftTurningEncoderReversed,
          24.455 - 355.484 - 50); //214.1, 158
          //getRobotSpecificOffset(0));
          

  private static final SwerveModule m_rearLeft =
      new SwerveModule(
          DriveConstants.kRearLeftDriveMotorPort,
          DriveConstants.kRearLeftTurningMotorPort,
          DriveConstants.kRearLeftTurningEncoderPorts,
          DriveConstants.kRearLeftDriveEncoderReversed,
          DriveConstants.kRearLeftTurningEncoderReversed,
          330.117 - 279.316 + 45 + 3); //-113, 16.5
          //getRobotSpecificOffset(1));

  private static final SwerveModule m_frontRight =
      new SwerveModule(
          DriveConstants.kFrontRightDriveMotorPort,
          DriveConstants.kFrontRightTurningMotorPort,
          DriveConstants.kFrontRightTurningEncoderPorts,
          DriveConstants.kFrontRightDriveEncoderReversed,
          DriveConstants.kFrontRightTurningEncoderReversed,
          79.013 - 0.615 - 135);//-153); //18.5, 250
          //getRobotSpecificOffset(2));

  private static final SwerveModule m_rearRight =
      new SwerveModule(
          DriveConstants.kRearRightDriveMotorPort,
          DriveConstants.kRearRightTurningMotorPort,
          DriveConstants.kRearRightTurningEncoderPorts,
          DriveConstants.kRearRightDriveEncoderReversed,
          DriveConstants.kRearRightTurningEncoderReversed,
          298.564 - 359.297 - 145);//-17.5, 30
          //getRobotSpecificOffset(3));

  // The gyro sensor
  //private final Gyro m_gyro = new ADXRS450_Gyro();
  public static AHRS m_gyro = new AHRS();

  Ultrasonic m_distanceUltrasonic = new Ultrasonic(4, 5);
  

  private final XboxController m_driveController = new XboxController(0);

  // Odometry class for tracking robot pose
  public static SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d(),
      new SwerveModulePosition[] { //do we need position
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public void enableUltrasonic() {
    Ultrasonic.setAutomaticMode(true);
    m_distanceUltrasonic.setEnabled(true);
    m_distanceUltrasonic.setAutomaticMode(true);
  }

  double defaultXSpeed = 0;
  double defaultYSpeed = 0;
  double defaultRot = 0;
  double startingPointX = 0;
  double startingPointY = 0;
  double destinationPointX = 0;
  double destinationPointY = 0;
  boolean autonMode = false;
  double lastX = 0;
  double lastY = 0;
  double lastZ = 0;


  

  @Override
  public void periodic() {
  enableUltrasonic();
    // Update the odometry in the periodic block
    
      m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });

      drive(Inputs.driverPower,
          Inputs.driverStrafe,
          Inputs.driverTurn,
          Inputs.fieldCentric);
          
      if(true){ //Constants.DashboardSwitches.DriveDisplayOn){
        SmartDashboard.putNumber("Ultrasonic Distance", m_distanceUltrasonic.getRangeInches());
        SmartDashboard.putBoolean("Is US Enabled?", m_distanceUltrasonic.isEnabled());
        SmartDashboard.putBoolean("Is US Range Valid", m_distanceUltrasonic.isRangeValid());
        SmartDashboard.putNumber("periodic X", m_odometry.getPoseMeters().getX() );
        SmartDashboard.putNumber("periodic Y", m_odometry.getPoseMeters().getY() );
        SmartDashboard.putNumber("periodic rot", m_odometry.getPoseMeters().getRotation().getDegrees() );
        SmartDashboard.putNumber("POV", m_driveController.getPOV());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
      }


  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void zeroGyro(){
    m_gyro.zeroYaw();
}

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
//    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    m_odometry.resetPosition(m_gyro.getRotation2d(),
    new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    },
    pose);
  }

  public static double getCubePower(double dInPower){
    return dInPower * Math.abs(dInPower * dInPower);   // yes this is not needed to perserve the sign, just go with it. 
}

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    Pose2d currentPose = m_odometry.getPoseMeters();
    double currentX = currentPose.getX();
    double currentY = currentPose.getY();
    // Constants.telemetry.putNumber("DRIVE Gyro Roll", m_gyro.getRoll());

    //Shaun 2/6/23 - Wheels remain in last position without moving.
     /* 
    if (xSpeed + ySpeed + rot > 0.05) {
      lastX = xSpeed;
      lastY = ySpeed;
      lastZ = rot;
    }

    if (Math.abs(xSpeed + ySpeed + rot) <= 0.01) { 
      xSpeed = lastX * 0.001;
      ySpeed = lastY * 0.001;
      rot = lastZ * 0.001;
    } */

   /****************************************************/
   //SmartDashboard.putNumber(Inputs., currentY)
   // not critical display items
  if(true){ //Constants.DashboardSwitches.DriveDisplayOn
    SmartDashboard.putString("Gyro YAW", Double.toString((double)m_gyro.getYaw()));
    SmartDashboard.putString("Gyro PITCH", Double.toString((double)m_gyro.getPitch()));
    SmartDashboard.putString("Gyro ROLL", Double.toString((double)m_gyro.getRoll()));
    SmartDashboard.putBoolean("Cool Auton Mode", autonMode);
    SmartDashboard.putNumber("currentX", currentPose.getX());
    SmartDashboard.putNumber("currentY", currentPose.getY());
    SmartDashboard.putNumber("Current Position", m_frontLeft.getCurrentDriveTicks());
    SmartDashboard.putBoolean("Robot Mode", fieldRelative);
    SmartDashboard.putNumber("RR", m_rearRight.getPosition().distanceMeters);
    SmartDashboard.putNumber("FR", m_frontRight.getPosition().distanceMeters);
    SmartDashboard.putNumber("FL", m_frontLeft.getPosition().distanceMeters);
    SmartDashboard.putNumber("RL", m_rearLeft.getPosition().distanceMeters);
  }

   /* if (autonMode == true) {
      if ((currentX >= destinationPointX) && (currentY >= destinationPointY)) {
        autonMode = false;
        xSpeed = 0.0;
        ySpeed = 0.0;
      }
      else {
        autonMode = true;
        xSpeed = 0.1;
        ySpeed = 0.1;
      }
    }
    else {
      xSpeed = (xSpeed); //Smooths driving speed curve
      ySpeed = (ySpeed);

      if (Math.abs(xSpeed) <= 0.05 && Math.abs(ySpeed) <= 0.05 && Math.abs(rot) <= 0.05) {
        xSpeed = defaultXSpeed*0.001;
        ySpeed = defaultYSpeed*0.001;
        rot = defaultRot*0.001;
      } else {
        defaultXSpeed = xSpeed;
        defaultYSpeed = ySpeed;
        defaultRot = rot;
      }
    } */

    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed, -ySpeed, -rot, Rotation2d.fromDegrees((m_gyro.getYaw()))) //xspeed was positive
                : new ChassisSpeeds(-xSpeed, -ySpeed, -rot)); //xspeed was positive
    //SwerveDriveKinematics.desaturateWheelSpeeds(
        //swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    //SmartDashboard.putNumber("Rotation?", rot );
    m_frontLeft.setDesiredState(swerveModuleStates[1]); //Adjusted the ports for each motor to change which motor got which position for turning
    m_frontRight.setDesiredState(swerveModuleStates[0]); 
    m_rearLeft.setDesiredState(swerveModuleStates[3]); //3
    m_rearRight.setDesiredState(swerveModuleStates[2]);

    
    if(Inputs.driveResetGyro) {
      zeroGyro();
    }

    if(Inputs.driveToggleBrakeMode){
      m_frontLeft.changeBrakeMode();
      m_frontRight.changeBrakeMode();
      m_rearLeft.changeBrakeMode();
      m_rearRight.changeBrakeMode();
    }

    if(Inputs.driveToggleAutoPoseMode){
      if (autonMode == true) {
        autonMode = false;
      }
      else {
        autonMode = true;
        Pose2d ourPose = m_odometry.getPoseMeters();
        startingPointX = ourPose.getX();
        startingPointY = ourPose.getY();
        destinationPointX = startingPointX + 5000;
        destinationPointY = startingPointY + 5000;
      }

      

      
      /*Trajectory exampleTrajectory =
          TrajectoryGenerator.generateTrajectory(
              // Start at the origin facing the +X direction
              new Pose2d(0, 0, new Rotation2d(0)),
              // Pass through these two interior waypoints, making an 's' curve path
              List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
              // End 1 meters straight ahead of where we started, facing forward
              new Pose2d(1, 0, new Rotation2d(0)),
              // Pass config
              config);*/      
    }
  // var gyroAngle = Rotation2d.fromDegrees(-m_gyro.getAngle());

  //m_odometry.update(gyroAngle, m_frontLeft.getState(), m_frontRight.getState(),
      // m_rearLeft.getState(), m_rearRight.getState());
  } 
  
  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[1]); //Adjusted the ports for each motor to change which motor got which position for turning
    m_frontRight.setDesiredState(desiredStates[0]);
    m_rearLeft.setDesiredState(desiredStates[3]);
    m_rearRight.setDesiredState(desiredStates[2]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  
  
}
