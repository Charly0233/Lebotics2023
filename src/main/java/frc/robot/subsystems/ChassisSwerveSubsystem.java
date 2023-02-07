// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModuleNEO;
import frc.robot.Constants.ChassisMode;
import frc.robot.Constants.IMU;
import frc.robot.Constants.SwerveDriveChassis;



public class ChassisSwerveSubsystem extends SubsystemBase {
  
  private static ChassisMode chassis_mode;

  private final SwerveModuleNEO m_frontLeft = new SwerveModuleNEO(7, 8, 21, 5.7);
  private final SwerveModuleNEO m_frontRight = new SwerveModuleNEO(5, 6, 22, 5.8);
  private final SwerveModuleNEO m_backLeft = new SwerveModuleNEO(1, 2, 23, 3.02);
  private final SwerveModuleNEO m_backRight = new SwerveModuleNEO(3, 4, 24, 1.7);
 
  SwerveModulePosition[] swerve_module_position;
  private final SwerveDriveOdometry odometer; 

  public ChassisSwerveSubsystem() {
    chassis_mode = ChassisMode.MANUAL;
    swerve_module_position = new SwerveModulePosition[4];
    swerve_module_position[0] = m_frontLeft.getPosition();
    swerve_module_position[1] = m_frontRight.getPosition();
    swerve_module_position[2] = m_backLeft.getPosition();
    swerve_module_position[3] = m_backRight.getPosition();
    odometer = new SwerveDriveOdometry(SwerveDriveChassis.m_kinematics, new Rotation2d(0), swerve_module_position);
  }


  public void resetEncoders(){
    m_frontLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_backLeft.resetEncoders();
    m_backRight.resetEncoders();
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean useAbsoluteEncoder) {
    // if(chassis_mode != ChassisMode.MANUAL) return;

    var swerveModuleStates =
        SwerveDriveChassis.m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, new Rotation2d(IMU.getIMU().getGyroAngleX(), IMU.getIMU().getGyroAngleY()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
                
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveDriveChassis.kMaxSpeed);


    m_frontLeft.setDesiredState(swerveModuleStates[2], useAbsoluteEncoder);
    m_frontRight.setDesiredState(swerveModuleStates[3], useAbsoluteEncoder);
    m_backLeft.setDesiredState(swerveModuleStates[0], useAbsoluteEncoder);
    m_backRight.setDesiredState(swerveModuleStates[1], useAbsoluteEncoder);

    // SmartDashboard.putNumber("Swerve Front Left ", m_frontRight.getTurningPosition());
    // SmartDashboard.putNumber("Swerve Front Right ", m_frontRight.getTurningPosition());
    // SmartDashboard.putNumber("Swerve Back Left ", m_backLeft.getTurningPosition());
    // SmartDashboard.putNumber("Swerve Back Right ", m_backRight.getTurningPosition());
    
    // System.out.println(
    //   m_frontLeft.getAbsolutePosition()+" "+
    //   m_frontRight.getAbsolutePosition()+" "+
    //   m_backLeft.getAbsolutePosition()+" "+
    //   m_backRight.getAbsolutePosition()
    // );

    // System.out.println("Pos: "+m_frontLeft.getDriveDistance());

    // System.out.println(
    //   xSpeed +" "+
    //   ySpeed +" "+
    //   rot
    // );

    
  
  }

  public void initializeWheels(){
    for(int i = 0; i<400; i++){
      drive(-0.102, 0.0, 0.0, false, true);
    }
    resetEncoders();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometer.update(new Rotation2d(IMU.getIMU().getGyroAngleZ()) , swerve_module_position );
    SmartDashboard.putString("ODOMETER", odometer.toString());
  }

  public Pose2d getPose(){
    return odometer.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
    SwerveModulePosition[] swerve_module_position = new SwerveModulePosition[4];
    swerve_module_position[0] = m_frontLeft.getPosition();
    swerve_module_position[1] = m_frontRight.getPosition();
    swerve_module_position[2] = m_backLeft.getPosition();
    swerve_module_position[3] = m_backRight.getPosition();

    odometer.resetPosition(new Rotation2d(IMU.getIMU().getGyroAngleZ()), swerve_module_position, pose  );
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
  }


  PIDController center_object_pid = new PIDController(0.05, 0, 0);
  public void centerObjectLeftRight(double _measure, double _target){
    if(chassis_mode != ChassisMode.CENTEROBJECT) return;
    double speed = -center_object_pid.calculate(_measure, _target);
    drive(0, speed, 0, false, false);
    System.out.println("centering... " + _measure +"  -> "+ speed);

  }



  public void setChassisMode(ChassisMode _mode){
    chassis_mode = _mode;
  }


  
  PIDController drive_distance_pid = new PIDController(0.1, 0, 0.0013);
  public void driveDistanceInCm(double _distance){
    if(chassis_mode != ChassisMode.DRIVEDISTANCE) return;
    double error = 2;
    m_frontLeft.resetDriveEncoder();
    do{
      double current_distance = m_frontLeft.getDriveDistance();
      double speed = drive_distance_pid.calculate(current_distance, _distance);
      drive(speed, 0, 0, false, false);
      System.out.println("driving... " + current_distance +" to get to "+_distance+"  -> "+ speed);
      if( (current_distance > _distance - error && current_distance < _distance + error) || Constants.driver1.getBackButtonPressed()){
        m_frontLeft.resetDriveEncoder();
        chassis_mode = ChassisMode.MANUAL;
      }
      
    }while(chassis_mode == ChassisMode.DRIVEDISTANCE);


  }





}
