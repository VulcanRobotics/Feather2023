// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.StringArrayEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import frc.robot.Constants;

//import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.ModuleConstants;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;

import org.ejml.equation.ManagerFunctions.Input1;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
//import frc.robot.SwerveMath.AngleMath;

import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.XboxController;




public class SwerveModule {

  private int turnMotorPortGlobal;
  private int turnEncoderPortGlobal;
  private double turningEncoderOffsetGlobal;
  private double realTurn;
  private double degreesToTicks;
  private double Talon360 = 24576;
  final double Talon180 = Talon360/2;
  final double Talon90 = Talon360/4;
  final double Talon270 = Talon90*3;
  
  private int brakeMode = 0;
  
  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor; //changed from private

  private final CANCoder m_turningEncoder;
  private int m_driveDirection = 1;
  private double talonOffsetOnBoot; 

  private boolean strafeMode = false;

  

  private final XboxController m_driveController = new XboxController(0);

  private final PIDController m_drivePIDController =
      new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderPorts,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed,
      double turningEncoderOffsetInDegree) {

        
    if (Constants.whichRobot == "Heather") {
      Talon360 = ModuleConstants.kHeatherTalon360;
      degreesToTicks = Talon360/360;

    } else {
      Talon360 = ModuleConstants.kPerseverance360;
      degreesToTicks = Talon360/60;
    }
        
    
    m_driveMotor = new TalonFX(driveMotorChannel, "DriveSubsystemCANivore");

    m_turningMotor = new TalonFX(turningMotorChannel, "DriveSubsystemCANivore");

    m_turningMotor.configSupplyCurrentLimit(
      new SupplyCurrentLimitConfiguration(true,     // enabled      
                                        10,         // limit
                                        40,         // trigger amp threshold
                                        0.1));      // trigger threshold time seconds


    m_turningEncoder = new CANCoder(turningEncoderPorts, "DriveSubsystemCANivore");

    m_driveMotor.setInverted(driveEncoderReversed);

    m_driveMotor.setNeutralMode(NeutralMode.Brake);

    // Parameters for StatorCurrentLimitConfiguration:   enabled | Limit(amp) | Trigger Threshold(amp) | Trigger Threshold Time(s)
    // We can play around with the amp and time numbers for driver preference
    m_driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 25, 30, 1.0));

    /*m_driveMotor.configSupplyCurrentLimit(
      new SupplyCurrentLimitConfiguration(true,     // enabled      
                                        10,         // limit
                                        40,         // trigger amp threshold
                                        0.1));*/     // trigger threshold time seconds

    talonOffsetOnBoot = (m_turningMotor.getSelectedSensorPosition() + (m_turningEncoder.getAbsolutePosition()*68.267))%Talon360;
    //finds the offset between the CAN Coder zero and TalonFX zero in ticks; remains constant

    turnMotorPortGlobal = turningMotorChannel; //Globals are used for reference in Smartdashboard

    turnEncoderPortGlobal = turningEncoderPorts;

    turningEncoderOffsetGlobal = turningEncoderOffsetInDegree;

    m_turningEncoder.configMagnetOffset(turningEncoderOffsetInDegree);

    



        
    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    


    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.


    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    //m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState( m_driveMotor.getSelectedSensorVelocity(), 
    Rotation2d.fromDegrees(m_turningEncoder.getAbsolutePosition()));//m_turningMotor.getSelectedSensorPosition() / ModuleConstants.kHeatherDegreesToTicks ) );
    
    
    //getDegrees()); ////Gotta replace them zeros

    //return new SwerveModuleState(m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.get()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition();
  }


  private double getCurrentTicks(){
    return m_turningMotor.getSelectedSensorPosition();
  }

  private double getCANTicks(){
    return m_turningEncoder.getAbsolutePosition() * 68.267;
  }

  private double getClosestZeroOriginal(SwerveModuleState desiredState) { //Currently unused; finds closest relative zero
    double turnOutput = desiredState.angle.getDegrees() * 68.267 + talonOffsetOnBoot;
    turnOutput = (turnOutput + (10* Talon360))%Talon360;
    double higherZero = (getCurrentTicks() + turnOutput) + (Talon360 - (getCurrentTicks()%Talon360));
    double lowerZero = (getCurrentTicks() + turnOutput) - (getCurrentTicks()%Talon360);

    if (Math.abs(getCurrentTicks() - lowerZero + turnOutput) <= Math.abs(higherZero - getCurrentTicks() + turnOutput)) {
      return lowerZero;
    }
    else {
      return higherZero;
    }
  } 

  private double getClosestZero(SwerveModuleState desiredState) { //Currently unused; finds closest relative zero
    double turnInDegrees = desiredState.angle.getDegrees();
    double currentTicks = getCurrentTicks();
    double currentTicksFirstTurn = currentTicks % Talon360; // converts to first turn
    
    if (turnInDegrees < 0) {
      turnInDegrees += 360;
    }

    double turnOutput = turnInDegrees * 68.267 + talonOffsetOnBoot;  // value in ticks
    turnOutput = turnOutput % Talon360;
    
    double first180 = turnOutput; //RENAME
    double second180 = turnOutput + Talon360;
    double oppositeFirst180 = turnOutput - Talon180;
    double oppositeSecond180 = turnOutput + Talon180;

    double winnerOne = 0.0;
    double winnerTwo = 0.0;
    double VICTORYROYALE = 0.0;
    
    if (Math.abs(first180 - currentTicksFirstTurn) <= Math.abs(second180 - currentTicksFirstTurn)) {
      winnerOne = first180;
    } else {
      winnerOne = second180;
    }

    if (Math.abs(oppositeFirst180 - currentTicksFirstTurn) <= Math.abs(oppositeSecond180 - currentTicksFirstTurn)) {
      winnerTwo = oppositeFirst180;
    } else {
      winnerTwo = oppositeSecond180;
    }

    if (Math.abs(winnerOne - currentTicksFirstTurn) <= Math.abs(winnerTwo - currentTicksFirstTurn)) {
      VICTORYROYALE = winnerOne;
      m_driveDirection = 1;
    } else {
      VICTORYROYALE = winnerTwo;
      m_driveDirection = -1;
    }

    return VICTORYROYALE + currentTicks - currentTicksFirstTurn;

 }

 public void changeBrakeMode() {
  if (brakeMode == 0) {
    brakeMode = 1;
    m_driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 0, 0, 0.0));
  }
  else if (brakeMode == 1) {
    brakeMode = 0;
    m_driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 25, 30, 1.0));
  }
}
   private double getOptimizedTurn(SwerveModuleState desiredState) { //Unused; finds an optimized turn value
    double combinedOffset = (desiredState.angle.getDegrees() * 68.267) + talonOffsetOnBoot;
    if (combinedOffset >= Talon360/2) {
      return (combinedOffset - Talon360);
    }
    else {
      return combinedOffset;
    }

  }
  


  


  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */



  public void setDesiredState(SwerveModuleState desiredState) {
 
     double driveOutput = -desiredState.speedMetersPerSecond * 1.0; //Uses desired state to return motor speed
    //double turnOutput = desiredState.angle.getDegrees() * 68.267 + talonOffsetOnBoot; //Finds desired turn for motors
      
   
    
    if (m_driveController.getStartButton()) {
       talonOffsetOnBoot = (m_turningMotor.getSelectedSensorPosition() + (m_turningEncoder.getAbsolutePosition()*68.267))%Talon360;
    }

    
      
      m_driveMotor.set(ControlMode.PercentOutput, driveOutput * m_driveDirection);
    //m_driveMotor.set(ControlMode.PercentOutput, driveOutput);
    m_turningMotor.set(ControlMode.Position, (getClosestZero(desiredState)));
    

    


    

 

    /* if (getCurrentTicks()%Talon360 >= Talon360/2) {
      realTurn = getCurrentTicks() + (Talon360 +(getCurrentTicks()%Talon360));
    }
    else {
      realTurn = getCurrentTicks() - (getCurrentTicks()%Talon360);
    }

    m_turningMotor.set(ControlMode.Position, desiredState.angle.getDegrees()*68.267 + talonOffsetOnBoot); */
    
     

    
    
    

  // 1 degree on CANCoder = 68.267 ticks on TalonFX

  // we want to go to the degree based on the cancoder


    SmartDashboard.putNumber("Desired Position for Wheel" +  Double.toString(turnMotorPortGlobal), desiredState.angle.getDegrees());
    SmartDashboard.putNumber("TALON OFFSET ON BOOT" + Integer.toString(turnMotorPortGlobal) , talonOffsetOnBoot);
    SmartDashboard.putNumber("Wheel " + Integer.toString(turnMotorPortGlobal) + " TalonFX Position", m_turningMotor.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Wheel " + Integer.toString(turnEncoderPortGlobal) + " CANCoder Position", m_turningEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("DESIRED TURN OUTPUT TICKS", realTurn);
    SmartDashboard.putNumber("DESIRED TURN OUTPUT DEGREES", realTurn/68.267);
    //SmartDashboard.putNumber("Closest Zero" + Integer.toString(turnMotorPortGlobal), getClosestZero(desiredState));
    //SmartDashboard.putNumber("Optimized Turn" + Integer.toString(turnMotorPortGlobal), getOptimizedTurn(desiredState));
    SmartDashboard.putNumber("Brake Mode", brakeMode);
    SmartDashboard.putNumber("Current Ticks" + Integer.toString(turnMotorPortGlobal), getCurrentTicks() ); 

    SmartDashboard.putNumber("m_driveDirection", m_driveDirection); 


  } 

  

  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {
    //m_turningEncoder.reset();
  } 
}
