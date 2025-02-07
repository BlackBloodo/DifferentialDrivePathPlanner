// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volt;

import java.util.Collection;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ChasisIds;
import frc.robot.Constants.chasisMeasurments;

public class DriveTrain extends SubsystemBase {
  private static DriveTrain m_instance;

  //Hardware
  private TalonFX m_leftLeader;
  private TalonFX m_leftFollower;
  private TalonFX m_rightLeader;
  private TalonFX m_rightFollower;
  private Pigeon2 m_gyro = new Pigeon2(13);
  private Orchestra musics = new Orchestra();
  private double Volts = 0;
  
  //Configurators
  private TalonFXConfiguration m_rightConfiguration = new TalonFXConfiguration();
  private TalonFXConfiguration m_leftConfiguration = new TalonFXConfiguration();
  private CurrentLimitsConfigs m_currentConfig = new CurrentLimitsConfigs();
  private final DutyCycleOut leftOut;
  private final DutyCycleOut rightOut;
  
  private final VoltageOut m_voltReq = new VoltageOut(0.0);
  private Encoder m_rightEncoder = new Encoder(1, 2);
  private Encoder m_leftEncoder = new Encoder(3, 4);
  

  edu.wpi.first.math.Vector<N3> quelems = VecBuilder.fill(0.125, 0.125, 3);
  edu.wpi.first.math.Vector<N2> relems = VecBuilder.fill(1, 2);
  //

  //Odometray
  private DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(0.525);
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.1392, 1.0281,0.12516);//, 0.3868);
  private PIDController LeftPIDController = new PIDController(1.0474, 0, 0);
  private PIDController righController = new PIDController(1.0474, 0, 0);

  //0.12035
  //0.1632
 /* private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.028061, 1.5,0.30931,0.005);//, 0.3868);
  private PIDController LeftPIDController = new PIDController(0.3503, 0, 0,0.005);
  private PIDController righController = new PIDController(0.3503, 0, 0,0.005);
 

  */
  private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), 0, 0);
  private RobotConfig config;

  public DriveTrain() {
    
    
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
    //Initialize Hardware
    m_leftLeader = new TalonFX(ChasisIds.k_leftTankDriveLeader);
    m_leftFollower = new TalonFX(ChasisIds.k_leftTankDriveFollower);
    m_rightLeader = new TalonFX(ChasisIds.k_rightTankDriveLeader);
    m_rightFollower = new TalonFX(ChasisIds.k_rightTankDriveFollower);
    m_gyro = new Pigeon2(ChasisIds.k_pygeon);

    

    musics.addInstrument(m_leftFollower,0);
    musics.addInstrument(m_rightFollower,1);
    musics.addInstrument(m_leftLeader,1);
    musics.addInstrument(m_rightLeader,2);
    var status = musics.loadMusic("outputkirby1.chrp");
    

    //Current Limit Configuration
    m_currentConfig.StatorCurrentLimitEnable = true;
    m_currentConfig.StatorCurrentLimit = 140;
    m_currentConfig.SupplyCurrentLimitEnable = true;
    m_currentConfig.SupplyCurrentLimit = 85;

    //Motor Configuration
    m_leftConfiguration.CurrentLimits = m_currentConfig;  
    m_leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_rightConfiguration.CurrentLimits = m_currentConfig;
    m_rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 
    m_leftLeader.getConfigurator().apply(m_leftConfiguration);
    m_leftFollower.getConfigurator().apply(m_leftConfiguration);
    m_rightFollower.getConfigurator().apply(m_rightConfiguration);
    m_rightFollower.getConfigurator().apply(m_rightConfiguration);
    

    m_leftLeader.setNeutralMode(NeutralModeValue.Brake);
    m_leftFollower.setNeutralMode(NeutralModeValue.Brake);
    m_rightLeader.setNeutralMode(NeutralModeValue.Brake);
    m_rightFollower.setNeutralMode(NeutralModeValue.Brake);

    leftOut = new DutyCycleOut(0);
    rightOut = new DutyCycleOut(0);
    
    /* Set up followers to follow leaders */    
    m_leftFollower.setControl(new Follower(m_leftLeader.getDeviceID(), false));
    m_rightFollower.setControl(new Follower(m_rightLeader.getDeviceID(), false));

    m_leftLeader.setSafetyEnabled(true);
    m_rightLeader.setSafetyEnabled(true);

    

    //Encoder setup
    m_rightEncoder.setDistancePerPulse(chasisMeasurments.encoderDistancePerPulse);
    m_leftEncoder.setDistancePerPulse(chasisMeasurments.encoderDistancePerPulse);

    m_rightEncoder.setReverseDirection(false);  
    m_leftEncoder.setReverseDirection(true);  


    //Odometry and Path Planner
    resetPosition();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

     try{
    AutoBuilder.configure(
      this::getPose, 
      this::resetPose, 
      this::getChassisSpeeds, 
      this::driveChassisSpeeds,
      new PPLTVController(0.02,4.8),
      config,
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      }, 
      this);
      }catch(Exception e){
        DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
      }
  }

  //Drive Function
  public void controlledDrive(double x, double y){
    double fwd = -x;
    double rot = y;
    leftOut.Output = fwd + rot;
    rightOut.Output = fwd - rot;
    m_leftLeader.setControl(leftOut);
    m_rightLeader.setControl(rightOut);
    m_leftLeader.feed();
    m_rightLeader.feed();
    m_rightFollower.feed();
    m_leftFollower.feed();
  }

  public static DriveTrain getInstance(){
    if(m_instance == null){
      m_instance = new DriveTrain();
    }
    return m_instance;
  }

  //Path Planner Functions
  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
    
  }

  public void resetPose(Pose2d newPose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), newPose);
  }

  public double getDistance(){
    // Return the process variable measurement here...
    double leftDistance = m_leftEncoder.getDistance();
    double rightDistance = m_rightEncoder.getDistance();
    return (leftDistance + rightDistance) / 2;
  }

  public ChassisSpeeds getChassisSpeeds(){
    var wheelSpeeds = new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
    return m_kinematics.toChassisSpeeds(wheelSpeeds);  
    
  }

  public void driveChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    
    DifferentialDriveWheelSpeeds speeds = m_kinematics.toWheelSpeeds(chassisSpeeds);
    
    final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);


    final double leftOutput =
        LeftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput =
        righController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);
    final VoltageOut leftvoltage = new VoltageOut(leftOutput + leftFeedforward);
    final VoltageOut righVoltage = new VoltageOut(rightOutput + rightFeedforward);
    m_leftLeader.setControl(leftvoltage);
    m_rightLeader.setControl(righVoltage);
    //m_leftLeader.setVoltage(leftOutput + leftFeedforward);
    //m_rightLeader.setVoltage(rightOutput + rightFeedforward);
  }

  //Odometry Functions
  public double PositionToMeters(double Position){
    Position = (Position * chasisMeasurments.wheelRaidusMeters / chasisMeasurments.gearRatio) * 2 * Math.PI;
    return Position;
  }

  public double VelocidadaMetros(double Velocidad){
    Velocidad *= chasisMeasurments.rotationVelocityToUnits;
    return Velocidad;
  }

  public double MetrosAVelocidad(double metros){
    metros *= chasisMeasurments.metrosToVelocity;
    return metros;
  }

  public void resetPosition(){
    m_leftLeader.setPosition(0);
    m_rightLeader.setPosition(0);
    m_gyro.setYaw(0);
    m_gyro.reset();
  }

  public Command rumbleControl(CommandXboxController controller){
    return runEnd(
        () -> {
          controller.setRumble(RumbleType.kBothRumble, 0.5);
          musics.play();
        
        },
        () -> {
          controller.setRumble(RumbleType.kBothRumble, 0);
          musics.stop();
        });
  }

  public void meterRumble(CommandXboxController control){
    while (getDistance() > 4 && getDistance() < 5) {
      control.setRumble(RumbleType.kBothRumble, 0.5);
    }
  }

  public Command Play(){
    return runOnce(
        () -> {
          musics.play();
        });
  }
    
  public Command stop(){
    return run(
        () -> {
          musics.stop();
        });
  }
  

  @Override
  public void periodic() {
   // This method will be called once per scheduler run
    SmartDashboard.putNumber("Angle", m_gyro.getYaw().getValueAsDouble());
    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    SmartDashboard.putNumber("LeftPosition", m_leftEncoder.getDistance());
    SmartDashboard.putNumber("RightPosition", m_rightEncoder.getDistance());
    SmartDashboard.putNumber("X", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Y", m_odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Velocidad", m_leftEncoder.getRate());
    SmartDashboard.putNumber("VOlts", m_leftLeader.getMotorVoltage().getValueAsDouble());
  }
}