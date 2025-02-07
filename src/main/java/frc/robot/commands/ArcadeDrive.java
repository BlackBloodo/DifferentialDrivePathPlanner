// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class ArcadeDrive extends Command {
private DoubleSupplier m_x;
  private DoubleSupplier m_y;

  double xfinal = 0;
  double yfinal = 0;

  boolean movimiento = false;


  /** Creates a new Drive. */
  public ArcadeDrive(DoubleSupplier _x, DoubleSupplier _y) {
    // Use addRequirements() here to declare subsystem dependencies.
    /*DoubleSupplier _x, DoubleSupplier _y BooleanSupplier _limit,*/
    /* 
    this.drive = drives;
    this.joy = control;*/
    addRequirements(DriveTrain.getInstance());
    
    this.m_x = _x;
    this.m_y = _y;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    
    double deadband_y = 0.09;
    double deadband_x = 0.09;
    
    double m_xDouble = m_x.getAsDouble();
    double m_yDouble = m_y.getAsDouble();

  
    
    if(m_xDouble >= 0 && m_xDouble < deadband_x){
      m_xDouble = 0;

      
    } 
     if(m_xDouble < 0 && m_xDouble > -deadband_x){
      m_xDouble = 0;

    } 
     if(m_yDouble >= 0 && m_yDouble < deadband_y){
      m_yDouble = 0;
      
    } 
     if(m_yDouble < 0 && m_yDouble > -deadband_y){
      m_yDouble = 0;
     }
    
    DriveTrain.getInstance().controlledDrive(m_xDouble , m_yDouble );
    

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
