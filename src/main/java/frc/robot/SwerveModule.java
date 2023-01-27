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

public class SwerveModule {
  public static final double kPhysicalMaxSpeedMetersPerSecond = 5;


  private static final double kModuleMaxAngularVelocity = SwerveDriveTrainChassis.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared
  // private static final double kWheelDiameterMeters = Units.inchesToMeters(4);
  // private static final double kDriveMotorGearRatio = 1 / 5.8462;
  private static final double kTurningMotorGearRatio = 1 / 14;
  // private static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
  private static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
  // private static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
  private static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
  private static final double kPTurning = 0.01;

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private RelativeEncoder m_driveEncoder;
  // private RelativeEncoder m_turningEncoder;

  CANCoder m_turningEncoder;


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
   * @param CANCoderId PWM output for the turning motor.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int CANCoderId
  ) {
    m_driveMotor = new CANSparkMax(driveMotorChannel,  MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel,  MotorType.kBrushless);

    // m_driveMotor.restoreFactoryDefaults();
    // m_turningMotor.restoreFactoryDefaults();

    m_driveEncoder = m_driveMotor.getEncoder();
    // m_turningEncoder = m_turningMotor.getEncoder();
    m_turningEncoder = new CANCoder(CANCoderId);
   

    // m_driveEncoder.setPositionConversionFactor(kDriveEncoderRot2Meter);
    // m_driveEncoder.setVelocityConversionFactor(kDriveEncoderRPM2MeterPerSec);
    // m_turningEncoder.setPositionConversionFactor(kTurningEncoderRot2Rad);
    // m_turningEncoder.setVelocityConversionFactor(kTurningEncoderRPM2RadPerSec);

    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    resetEncoders();

  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // return new SwerveModuleState( m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.getDistance()));
    return new SwerveModuleState( m_driveEncoder.getVelocity(), new Rotation2d(getTurningPosition()));

  }
  public void resetEncoders(){
    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition(0);
  }
  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        // m_driveEncoder.getDistance(), new Rotation2d(m_turningEncoder.getDistance()));
        getDrivePosition(), new Rotation2d(getTurningPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    if(Math.abs(desiredState.speedMetersPerSecond) < 0.01){
      stop();
      return;
    }

    desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
    m_driveMotor.set(   desiredState.speedMetersPerSecond / kPhysicalMaxSpeedMetersPerSecond);
    double turn_speed = m_turningPIDController.calculate(getTurningPosition(), desiredState.angle.getRadians() ) ;
    m_turningMotor.set(   turn_speed   );
    


    SmartDashboard.putNumber("Swerve turn speed["+m_turningMotor.getDeviceId()+"] state", turn_speed);
    SmartDashboard.putNumber("Swerve["+m_turningMotor.getDeviceId()+"] state",getTurningPosition());
    SmartDashboard.putNumber("Swerve desired["+m_turningMotor.getDeviceId()+"] state", desiredState.angle.getRadians());

  }



  public double getDrivePosition(){
    return m_driveEncoder.getPosition();
  }
  public double getTurningPosition(){
    // return m_turningEncoder.getPosition();
    return m_turningEncoder.getPosition() * Math.PI / 180;
    // return m_turningEncoder.getAbsolutePosition() * Math.PI / 90;

  }
  public void stop(){
    m_driveMotor.set(0);
    m_turningMotor.set(0);
  }


}
