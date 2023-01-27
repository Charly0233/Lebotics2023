// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static XboxController driver1 = new XboxController(0);



  public static class SwerveDriveTrainChassis{
    public static final double kMaxSpeed = 3.0; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }





  
  public static class IMU{
    private static final ADIS16448_IMU imu = new ADIS16448_IMU();
    public IMU(){
      imu.calibrate();
    }
    public static ADIS16448_IMU getIMU(){
      return imu;
    }
  }
  
}
