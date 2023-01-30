// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightBase extends SubsystemBase {

  public double pX = -0.01;
  public double fX = 0.05;
  public double pDistance = 0.1;

  NetworkTable m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = m_limelightTable.getEntry("tx");
  NetworkTableEntry ty = m_limelightTable.getEntry("ty");
  NetworkTableEntry ta = m_limelightTable.getEntry("ta");
  NetworkTableEntry tv = m_limelightTable.getEntry("tv");

  public double x;
  public double y;
  public double a;
  public double v;

  /** Creates a new LimelightBase. */
  public LimelightBase() {

  }
   public void setTracking(boolean tracking) {
        m_limelightTable.getEntry("camMode").setNumber(tracking ? 0 : 1);
        m_limelightTable.getEntry("ledMode").setNumber(tracking ? 0 : 1);

   }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    a = ta.getDouble(0.0);
    v = tv.getDouble(0.0);

    
  }



}
