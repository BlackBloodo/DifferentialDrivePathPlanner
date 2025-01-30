// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.util.DriveFeedforwards;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  //Configurators
  private TalonFXConfiguration m_righConfiguration = new TalonFXConfiguration();
  private TalonFXConfiguration m_leftConfiguration = new TalonFXConfiguration();
  private CurrentLimitsConfigs m_currentConfig = new CurrentLimitsConfigs();
  private final DutyCycleOut leftOut;
  private final DutyCycleOut rightOut;
  private final VelocityDutyCycle leftVelocityDutyCycle;
  private final VelocityDutyCycle righVelocityDutyCycle;
  private Encoder rightEncoder = new Encoder(1, 2);
  private Encoder leftEncoder = new Encoder(3, 4);
  

  edu.wpi.first.math.Vector<N3> quelems = VecBuilder.fill(0.25, 0.25, 8);
  edu.wpi.first.math.Vector<N2> relems = VecBuilder.fill(1, 2);
  //

  //Odometry
  private DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(0.546);
  private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0.1395, 1.0281,0.12516);
  private PIDController m_leftPIDController = new PIDController(1.0474, 0, 0);
  private PIDController m_rightPIDController = new PIDController(1.0474, 0, 0);

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

    //Current Limit Configuration
    m_currentConfig.StatorCurrentLimitEnable = true;
    m_currentConfig.StatorCurrentLimit = 140;
    m_currentConfig.SupplyCurrentLimitEnable = true;
    m_currentConfig.SupplyCurrentLimit = 80;

    //Motor Configuration
    m_leftConfiguration.CurrentLimits = m_currentConfig;  
    m_leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_righConfiguration.CurrentLimits = m_currentConfig;
    m_righConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 
    m_leftLeader.getConfigurator().apply(m_leftConfiguration);
    m_leftFollower.getConfigurator().apply(m_leftConfiguration);
    m_rightFollower.getConfigurator().apply(m_righConfiguration);
    m_rightFollower.getConfigurator().apply(m_righConfiguration);

    m_leftLeader.setNeutralMode(NeutralModeValue.Brake);
    m_leftFollower.setNeutralMode(NeutralModeValue.Brake);
    m_rightLeader.setNeutralMode(NeutralModeValue.Brake);
    m_rightFollower.setNeutralMode(NeutralModeValue.Brake);

    leftOut = new DutyCycleOut(0);
    rightOut = new DutyCycleOut(0);
    leftVelocityDutyCycle = new VelocityDutyCycle(0);
    righVelocityDutyCycle = new VelocityDutyCycle(0);
    
    /* Set up followers to follow leaders */    
    m_leftFollower.setControl(new Follower(m_leftLeader.getDeviceID(), false));
    m_rightFollower.setControl(new Follower(m_rightLeader.getDeviceID(), false));

    m_leftLeader.setSafetyEnabled(true);
    m_rightLeader.setSafetyEnabled(true);

    leftEncoder.setDistancePerPulse(0.000005844466802);
    rightEncoder.setDistancePerPulse(0.000005844466802);
    

    //Odometry and Path Planner
    resetPosition();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), PositionToMeters(m_leftLeader.getPosition().getValueAsDouble()), PositionToMeters(m_rightLeader.getPosition().getValueAsDouble()));

     try{
    AutoBuilder.configure(
      this::getPose, 
      this::resetPose, 
      this::getChassisSpeeds, 
      (speeds, feedforward) -> driveChassisSpeeds(speeds, feedforward),
      new PPLTVController(quelems, relems, 0.02),
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
    m_odometry.resetPosition(m_gyro.getRotation2d(), PositionToMeters(m_leftLeader.getPosition().getValueAsDouble()), PositionToMeters(m_rightLeader.getPosition().getValueAsDouble()), newPose);
  }

  public ChassisSpeeds getChassisSpeeds(){
    var wheelSpeeds = new DifferentialDriveWheelSpeeds(VelocidadaMetros(m_leftLeader.getVelocity().getValueAsDouble()), VelocidadaMetros(m_rightLeader.getVelocity().getValueAsDouble()));
    return m_kinematics.toChassisSpeeds(wheelSpeeds);  
    
  }

  public void driveChassisSpeeds(ChassisSpeeds chassisSpeeds, DriveFeedforwards feedforwards) {
    DifferentialDriveWheelSpeeds speeds = m_kinematics.toWheelSpeeds(chassisSpeeds);
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput =
        m_leftPIDController.calculate(VelocidadaMetros(m_leftLeader.getVelocity().getValueAsDouble()), speeds.leftMetersPerSecond);
    final double rightOutput =
        m_rightPIDController.calculate(VelocidadaMetros(m_rightLeader.getVelocity().getValueAsDouble()), speeds.rightMetersPerSecond);
    m_leftLeader.setVoltage(leftOutput + leftFeedforward);
    m_rightLeader.setVoltage(rightOutput + rightFeedforward);
    
    /*leftVelocityDutyCycle.Velocity = VelocidadaMetros(speeds.leftMetersPerSecond);
    righVelocityDutyCycle.Velocity = VelocidadaMetros(speeds.rightMetersPerSecond);
    //righVelocityDutyCycle.Acceleration = (feedforwards.accelerationsMPSSq()[1]) / 0.0762 * 8.45;
    //leftVelocityDutyCycle.Acceleration = (feedforwards.accelerationsMPSSq()[0]) / 0.0762 * 8.45;
    m_leftLeader.setControl(leftVelocityDutyCycle);
    m_rightLeader.setControl(righVelocityDutyCycle);*/
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

  @Override
  public void periodic() {
   // This method will be called once per scheduler run
    SmartDashboard.putNumber("Angle", m_gyro.getYaw().getValueAsDouble());
    m_odometry.update(m_gyro.getRotation2d(), PositionToMeters(m_leftLeader.getPosition().getValueAsDouble()), PositionToMeters(m_rightLeader.getPosition().getValueAsDouble()));
    SmartDashboard.putNumber("LeftPosition", m_leftLeader.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("RightPosition", m_rightLeader.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("X", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Y", m_odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Velocidad", VelocidadaMetros(m_leftFollower.getVelocity().getValueAsDouble()));
  }
}