// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ChasisIds {
    public static final int k_leftTankDriveLeader = 4; 
    public static final int k_leftTankDriveFollower = 5;
    public static final int k_rightTankDriveLeader = 2;  
    public static final int k_rightTankDriveFollower = 3;
    public static final int k_pygeon = 13;
  }


  public static class chasisMeasurments {

    public static final double encoderCPR = 2048;
    public static final double wheelDiameterMeters = Units.inchesToMeters(6);
    public static final double encoderDistancePerPulse = -(Math.PI * wheelDiameterMeters) / encoderCPR;

    
    public static final double wheelRaidusMeters = 0.0762;
    public static final double gearRatio = 8.45;
    public static final double rotationVelocityToUnits = (3.14159265 *2 * wheelRaidusMeters / gearRatio);
    public static final double metrosToVelocity = (gearRatio / (3.14159265 * 2 * gearRatio));
  }

 
}
