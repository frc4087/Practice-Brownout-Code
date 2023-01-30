// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

//import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;



public class DriveBase extends SubsystemBase {

  //public WPI_TalonFX _left1 = new WPI_TalonFX(2);
  /*public WPI_TalonFX _left2 = new WPI_TalonFX(3);
  public WPI_TalonFX _left3 = new WPI_TalonFX(4);


  public WPI_TalonFX _right1 = new WPI_TalonFX(5);
  public WPI_TalonFX _right2 = new WPI_TalonFX(6);
  public WPI_TalonFX _right3 = new WPI_TalonFX(7);


  /** Creates a new DriveBase. */

  public CANSparkMax _left1 = new CANSparkMax(4, MotorType.kBrushless);
  public CANSparkMax _left2 = new CANSparkMax(3, MotorType.kBrushless);

  public CANSparkMax _right1 = new CANSparkMax(2, MotorType.kBrushless);
  public CANSparkMax _right2 = new CANSparkMax(5, MotorType.kBrushless);

  public DifferentialDrive m_drive = new DifferentialDrive(_left1, _right1);

  public DriveBase() {

    //Creates Leader-Follower relationship between motors 
    _left2.follow(_left1);
    _right2.follow(_right1);

    //Inverts left motors from standard

    _left1.setInverted(true);
    _left2.setInverted(true);

   _right1.setInverted(false);
   _right2.setInverted(false);
  

    //Sets motors to Brake mode!!!!!!!!
    //Eady is a poopy stupid head

    _left1.setIdleMode(IdleMode.kBrake);
    _left2.setIdleMode(IdleMode.kBrake);

    _right1.setIdleMode(IdleMode.kBrake);
    _right2.setIdleMode(IdleMode.kBrake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
