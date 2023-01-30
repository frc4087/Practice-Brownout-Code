// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.DriveBaseTalon;
import frc.robot.subsystems.IntakePrototype;
import frc.robot.subsystems.LimelightBase;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.networktables.IntegerArrayEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  //public final DriveBase m_DriveBase = new DriveBase();
  public final DriveBaseTalon m_driveBaseTalon = new DriveBaseTalon();
  public final LimelightBase m_limelightBase = new LimelightBase();
  public static final IntakePrototype m_IntakePrototype = new IntakePrototype();

  //public final IntakePrototype m_intakePrototype = new IntakePrototype();

  //public DoubleSolenoid switchSol = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 0, 1);

  public final XboxController driveJoy = new XboxController(0);
  public final XboxController opJoy = new XboxController(1);
  public final JoystickButton aButton = new JoystickButton(opJoy,1);
  public final JoystickButton bButton = new JoystickButton(opJoy,2);
  public final JoystickButton startButton = new JoystickButton(opJoy,8);
  //String trajectoryJSON = "paths/YourPath.wpilib.json";
  public Command m_autonomousCommand;
  public Trajectory trajectory;
  public SendableChooser<String> autoChooser = new SendableChooser<String>();


  public double getDriveJoy(int axis) {
    double raw = driveJoy.getRawAxis(axis);
    return Math.abs(raw) < 0.1 ? 0.0 : raw;
  }
  public double getDriveJoyXR() {
    double raw = getDriveJoy(4);
    return raw;
    //return Math.abs(raw) < 0.1 ? 0.0 : raw > 0 ? (raw * raw) / 1.5 : (-raw * raw) / 1.5;
  }

  public double getDriveJoyYL() {
    double raw = getDriveJoy(1);
    return raw;
    //return Math.abs(raw) < 0.1 ? 0.0 : raw > 0 ? (raw * raw) / 1.5 : (-raw * raw) / 1.5;
  }
  
  public void robotInit(){
    autoChooser.addOption("Mobility1","Mobility1");
    autoChooser.addOption("Mobility2","Mobility2");
    autoChooser.addOption("Mobility2Dock","Mobility2Dock");
    autoChooser.addOption("Mobility3","Mobility3");

    SmartDashboard.putData("Auto Routine", autoChooser);
  }

  public void robotPeriodic(){
    SmartDashboard.putNumber("Drive Encoder Left", m_driveBaseTalon.leftEncPos);
    SmartDashboard.putNumber("Drive Encoder Right", m_driveBaseTalon.rightEncPos);
   
    SmartDashboard.putNumber("Drive PO Left", m_driveBaseTalon._left1.getMotorOutputPercent());
    SmartDashboard.putNumber("Drive PO Right", m_driveBaseTalon._right1.getMotorOutputPercent());
    SmartDashboard.putNumber("Gyro", m_driveBaseTalon.m_gyro.getYaw());
  }

  public void autoInit(){
    //m_IntakeBase.intakeSol1.set(Value.kForward);
    //m_BlinkinBase.set(Constants.autoIdle);
    m_driveBaseTalon.resetEncoders();
    m_driveBaseTalon.m_gyro.reset();
    m_driveBaseTalon.resetOdometry(null);

    if (autoChooser.getSelected() != null){
      m_autonomousCommand = getAutonomousCommand(autoChooser.getSelected());
    //  m_driveBaseTalon.resetOdometry(trajectory.getInitialPose());
      m_autonomousCommand.schedule();
    }

  }

  public void autoPeriodic(){
    SmartDashboard.putNumber("Pose X", m_driveBaseTalon.m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Pose Y", m_driveBaseTalon.m_odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Pose Angle", m_driveBaseTalon.m_odometry.getPoseMeters().getRotation().getDegrees());
  }

  public void teleopInit(){
    //switchSol.set(Value.kForward);
    
  }

  
  public void teleopPeriodic(){

    SmartDashboard.putNumber("LimelightX", m_limelightBase.x);
    SmartDashboard.putNumber("LimelightY", m_limelightBase.y);
    SmartDashboard.putNumber("LimelightArea", m_limelightBase.a);
    SmartDashboard.putNumber("LimelightValid", m_limelightBase.v);

    //Drivebase
    //try arcade drive
    if (driveJoy.getXButtonPressed()){
        m_limelightBase.setTracking(true);
        double steeringAdjust = 0.38 * m_limelightBase.pX;
        m_driveBaseTalon.m_drive.arcadeDrive(getDriveJoyYL() * m_limelightBase.v,steeringAdjust);
      // double x_error = m_limelightBase.x;
      // double distance_error = 20-m_limelightBase.a;
      // double steering_adjust = 0.0;
      // double left_command = 0;
      // double right_command = 0;

      // if (m_limelightBase.x > 1.0){

      //   steering_adjust = m_limelightBase.pX*x_error; //- m_limelightBase.fX;

      // } else if (m_limelightBase.x < 1.0) {

      //   steering_adjust = m_limelightBase.pX*x_error; //+ m_limelightBase.fX;

      // }

      // steering_adjust = m_limelightBase.pX*x_error;

      // double distance_adjust = m_limelightBase.pDistance * distance_error;

      // left_command += steering_adjust + distance_adjust;
      // right_command -= steering_adjust + distance_adjust;

      // m_driveBaseTalon.m_drive.tankDrive(left_command, right_command);

    } else {

   //   m_DriveBase.m_drive.arcadeDrive(getDriveJoyYL(), getDriveJoyXR());
      m_driveBaseTalon.m_drive.arcadeDrive(getDriveJoyYL(), getDriveJoyXR());

    }
  
    // if(opJoy.getBButtonPressed()){
    //   IntakePrototype.pSpark.set(100);
    // }else if(opJoy.getAButtonPressed()){
    //   IntakePrototype.pSpark.set(100);
    // }

    // if(opJoy.getLeftBumper()){
    //   m_IntakePrototype.pSpark.set(1000);
    // } else if(opJoy.getRightBumper()){
    //   m_IntakePrototype.pSpark.set(-1000);
    // } else {
    //   m_IntakePrototype.pSpark.set(0.0);
    // }
 
    
/* 
    //Solenoid Trigger
    if (driveJoy.getAButtonPressed()) {
      switchSol.toggle();
    }

    */

    //Intake motor
   // m_intakePrototype.pSpark.set(getDriveJoyXR());
  
   }


   public Command pathFollow(String trajectoryJSON, boolean multiPath){

    try {
      Path testTrajectory = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(testTrajectory);
    } catch (final IOException ex) {
      // TODO Auto-generated catch block
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

   

    
    RamseteCommand ramseteCommand = new RamseteCommand(trajectory,
                                                    m_driveBaseTalon::getPose,
                                                    new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                                                    new SimpleMotorFeedforward(Constants.ksVolts, 
                                                                               Constants.kvVoltSecondsPerMeter,
                                                                               Constants.kaVoltSecondsSquaredPerMeter),
                                                    Constants.m_driveKinematics,
                                                    m_driveBaseTalon::getWheelSpeeds,
                                                    new PIDController(Constants.kP, 0, 0),
                                                    new PIDController(Constants.kP, 0, 0),
                                                    m_driveBaseTalon::voltageControl,
                                                    m_driveBaseTalon);
    
    // Run path following command, then stop at the end.
   
    //m_driveBaseTalon.resetOdometry(trajectory.getInitialPose());

    if (!multiPath){
      m_driveBaseTalon.resetOdometry(trajectory.getInitialPose());
    } 

    return ramseteCommand;
  }


  public Command getAutonomousCommand(String path) {
    switch(path){
    case "Mobility1":
      return pathFollow("output/Mobility1.wpilib.json", false);
    case "Mobility2":
      return pathFollow("output/Mobility2.wpilib.json", false);
    case "Mobility2Dock":
      return pathFollow("output/Mobility2.wpilib.json", false)
              .andThen(pathFollow("output/Mobility2Rev.wpilib.json", true));
     case "Mobility3":
      return pathFollow("output/Mobility3.wpilib.json", false);
      
    }
    return null;
    }
 /** The container for the robot. Contains subsystems, OI devices, and commands. */

  public RobotContainer() {


  }
}

