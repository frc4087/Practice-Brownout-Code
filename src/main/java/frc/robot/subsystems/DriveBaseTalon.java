// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveBaseTalon extends SubsystemBase {

  public WPI_TalonFX _left1 = new WPI_TalonFX(2);
  public WPI_TalonFX _left3 = new WPI_TalonFX(4);


  public WPI_TalonFX _right1 = new WPI_TalonFX(5);
  public WPI_TalonFX _right3 = new WPI_TalonFX(7);
  
  public double leftEncPos; //= _left1.getSelectedSensorPosition() * Constants.kEncDistPerPulse; 
  public double rightEncPos; //= _right1.getSelectedSensorPosition() * Constants.kEncDistPerPulse;
  public double leftEncVel; //= _left1.getSelectedSensorVelocity() *10* Constants.kEncDistPerPulse;
  public double rightEncVel; // = _right1.getSelectedSensorVelocity() *10* Constants.kEncDistPerPulse;

  public DifferentialDrive m_drive = new DifferentialDrive(_left1, _right1);

 // public AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  public WPI_Pigeon2 m_gyro = new WPI_Pigeon2(0);

  
  public DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), leftEncPos, rightEncPos);
  //Eady is a poopy stupid head
  /** Creates a new DriveBaseTalon. */
  public DriveBaseTalon() {

   
    
    //Creates Leader-Follower relationship between motors 
    
    _left3.follow(_left1);
    _right3.follow(_right1);

    //Sets right motors to inverted position

    _left1.setInverted(false);
    _left3.setInverted(false);

   _right1.setInverted(true);
   _right3.setInverted(true);

   _left1.setNeutralMode(NeutralMode.Brake);
   _left3.setNeutralMode(NeutralMode.Brake);

   _right1.setNeutralMode(NeutralMode.Brake);
   _right3.setNeutralMode(NeutralMode.Brake);

   _left1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,30);
   _right1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,30);

   _left1.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 30, 25, 0.8));
   _left3.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 30, 25, 0.8));
   _right1.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 30, 25, 0.8));
   _right3.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 30, 25, 0.8));

   _left1.setStatusFramePeriod(1, 20);
   _right1.setStatusFramePeriod(1, 20);
   _left3.setStatusFramePeriod(1, 20);
   _right3.setStatusFramePeriod(1, 20);

   _left1.setStatusFramePeriod(3, 13100);
   _right1.setStatusFramePeriod(3, 13200);
   _left3.setStatusFramePeriod(3, 13500);
   _right3.setStatusFramePeriod(3, 13600);

   _left1.setStatusFramePeriod(4, 17100);
   _right1.setStatusFramePeriod(4, 17200);
   _left3.setStatusFramePeriod(4, 17500);
   _right3.setStatusFramePeriod(4, 17600);

   _left1.setStatusFramePeriod(8, 19100);
   _right1.setStatusFramePeriod(8, 19200);
   _left3.setStatusFramePeriod(8, 19500);
   _right3.setStatusFramePeriod(8, 19600);

   _left1.setStatusFramePeriod(14, 23100);
   _right1.setStatusFramePeriod(14, 23200);
   _left3.setStatusFramePeriod(14, 23500);
   _right3.setStatusFramePeriod(14, 23600);

    


  }

  @Override
  public void periodic() {
    //Removed *10 
    leftEncPos = _left1.getSelectedSensorPosition() * Constants.kEncDistPerPulse; 
    rightEncPos = _right1.getSelectedSensorPosition() * Constants.kEncDistPerPulse;
    leftEncVel = _left1.getSelectedSensorVelocity() * Constants.kEncDistPerPulse;
    rightEncVel = _right1.getSelectedSensorVelocity() * Constants.kEncDistPerPulse;
    
    //m_odometry.update(m_gyro.getRotation2d(), leftEncPos, rightEncPos);
    //m_odometry.resetPosition(m_gyro.getRotation2d(), leftEncPos, rightEncPos, getPose());
    
    // This method will be called once per scheduler run
  }
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }


  public void resetEncoders() {
    leftEncPos = 0;
    rightEncPos = 0;
  }

  public void resetOdometry(final Pose2d pose) {
    resetEncoders();
    zeroHeading();
    m_odometry.resetPosition(m_gyro.getRotation2d(), leftEncPos, rightEncPos, getPose());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {    
    return new DifferentialDriveWheelSpeeds(leftEncVel, rightEncVel);
  }
  

  public void zeroHeading() {
    m_gyro.reset();
  }

  public double getOtherHeading(){
    return m_gyro.getPitch();
  }

  
  public double getAnotherHeading(){
    return m_gyro.getRoll();
  }

  
  public void voltageControl(final double leftVolts, final double rightVolts) {
    _left1.setVoltage(leftVolts); 
    _right1.setVoltage(rightVolts); //used to be negative
    m_drive.feed(); 
  }
}
