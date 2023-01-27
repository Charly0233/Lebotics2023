// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveDriveTrainChassis;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class SwerveModuleNEO {
  public static final double kPhysicalMaxSpeedMetersPerSecond = 5;

  private static final double kModuleMaxAngularVelocity = SwerveDriveTrainChassis.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared
  private static final double kTurningMotorGearRatio = 1 / 14.0;
  private static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
  private static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
  private static final double kPTurning = 0.1;

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private RelativeEncoder m_driveEncoder;
  private RelativeEncoder m_turningEncoder;



  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          kPTurning,
          0,
          0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));


  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   */
  public SwerveModuleNEO(
      int driveMotorChannel,
      int turningMotorChannel
  ) {
    m_driveMotor = new CANSparkMax(driveMotorChannel,  MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel,  MotorType.kBrushless);

    m_driveMotor.restoreFactoryDefaults();
    m_turningMotor.restoreFactoryDefaults();

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningEncoder = m_turningMotor.getEncoder();
   

    // m_driveEncoder.setPositionConversionFactor(kDriveEncoderRot2Meter);
    // m_driveEncoder.setVelocityConversionFactor(kDriveEncoderRPM2MeterPerSec);
    m_turningEncoder.setPositionConversionFactor(kTurningEncoderRot2Rad);
    m_turningEncoder.setVelocityConversionFactor(kTurningEncoderRPM2RadPerSec);

    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    resetEncoders();

  }


  public double getDrivePosition(){
    return m_driveEncoder.getPosition();
}

public double getTurningPosition(){
    return m_turningEncoder.getPosition();

}

public double getDriveVelocity(){
    return m_driveEncoder.getVelocity();
}

public double getTurningVelocity(){
    return m_turningEncoder.getVelocity();

}



public void resetEncoders(){
    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition(0);
}




public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
}


public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
}

public void setDesiredState(SwerveModuleState _state){
  if(Math.abs(_state.speedMetersPerSecond) < 0.001){
        stop();
        return;
    }

    _state = SwerveModuleState.optimize(_state, getState().angle);
    m_driveMotor.set(_state.speedMetersPerSecond / kPhysicalMaxSpeedMetersPerSecond);
    m_turningMotor.set(m_turningPIDController.calculate(getTurningPosition(), _state.angle.getRadians()  ));

  SmartDashboard.putNumber("Swerve desired["+m_turningMotor.getDeviceId()+"] state", _state.angle.getRadians());

  }
public void stop(){
    m_driveMotor.set(0);
    m_turningMotor.set(0);
}

}
