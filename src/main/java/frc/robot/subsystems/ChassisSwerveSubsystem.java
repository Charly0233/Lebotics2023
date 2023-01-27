// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModuleNEO;
import frc.robot.Constants.IMU;
import frc.robot.Constants.SwerveDriveTrainChassis;

public class ChassisSwerveSubsystem extends SubsystemBase {
  
  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModuleNEO m_frontLeft = new SwerveModuleNEO(7, 8);
  private final SwerveModuleNEO m_frontRight = new SwerveModuleNEO(5, 6);
  private final SwerveModuleNEO m_backLeft = new SwerveModuleNEO(1, 2);
  private final SwerveModuleNEO m_backRight = new SwerveModuleNEO(3, 4);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics( m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  /** Creates a new ChassisSwerveSubsystem. */
  public ChassisSwerveSubsystem() {}

  
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, new Rotation2d(IMU.getIMU().getGyroAngleX(), IMU.getIMU().getGyroAngleY()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveDriveTrainChassis.kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[2]);
    m_frontRight.setDesiredState(swerveModuleStates[3]);
    m_backLeft.setDesiredState(swerveModuleStates[0]);
    m_backRight.setDesiredState(swerveModuleStates[1]);



    SmartDashboard.putNumber("Swerve Front Left ", m_frontRight.getTurningPosition());
    SmartDashboard.putNumber("Swerve Front Right ", m_frontRight.getTurningPosition());
    SmartDashboard.putNumber("Swerve Back Left ", m_backLeft.getTurningPosition());
    SmartDashboard.putNumber("Swerve Back Right ", m_backRight.getTurningPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
