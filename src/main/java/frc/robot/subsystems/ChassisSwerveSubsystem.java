// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModuleNEO;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.ChassisMode;
import frc.robot.Constants.IMURobot;
import frc.robot.Constants.InitializeWheels;
import frc.robot.Constants.SwerveDriveChassis;



public class ChassisSwerveSubsystem extends SubsystemBase {
  
  private static ChassisMode chassis_mode;

  private final SwerveModuleNEO m_frontLeft = new SwerveModuleNEO(1, 2, 21, 5.7);
  private final SwerveModuleNEO m_frontRight = new SwerveModuleNEO(5, 6, 23, 2.78);
  private final SwerveModuleNEO m_backLeft = new SwerveModuleNEO(3, 4, 22, 0.84);
  private final SwerveModuleNEO m_backRight = new SwerveModuleNEO(7, 8, 24, 1.43);

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
    // initializeWheels();
  }

  boolean enable_turbo = false;
  public void enableTurbo(boolean _actuator){
    if(_actuator){
      enable_turbo = !enable_turbo;
    }
    if(enable_turbo){
      SwerveDriveChassis.setMaxSpeed(5);
      SmartDashboard.putBoolean("Turbo", true);
    }else{
      SwerveDriveChassis.setMaxSpeed(2);
      SmartDashboard.putBoolean("Turbo", false);
    }

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
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, new Rotation2d(IMURobot.getPitch(), IMURobot.getRoll()))
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

  public void initializeWheels(InitializeWheels _mode){
    for(int i = 0; i<400; i++){
      if(_mode == InitializeWheels.VERTICAL){
        drive(-0.15, 0.0, 0.0, false, true); //-0.102
      }
      else if(_mode == InitializeWheels.VERTICAL){
        drive(0.0, -0.15, 0.0, false, true); //-0.102
      }
    }
    resetEncoders();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometer.update(new Rotation2d(IMURobot.getPitch()) , swerve_module_position );
    // System.out.println("ROBOT Roll: "+ IMURobot.getRoll()+"Pitch:"+IMURobot.getPitch() +"Yaw: "+IMURobot.getPitch());

    // System.out.println("Front LEFT: "+m_frontLeft.getAbsolutePosition());
    // System.out.println("Front RIGHT: "+m_frontRight.getAbsolutePosition());
    // System.out.println("Back LEFT: "+m_backLeft.getAbsolutePosition());
    // System.out.println("Back RIGHT: "+m_backRight.getAbsolutePosition());
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

    odometer.resetPosition(new Rotation2d(IMURobot.getYaw()), swerve_module_position, pose  );
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
  }


  PIDController center_object_pid = new PIDController(0.05, 0, 0);
  public void centerObjectLeftRight(double _measure, double _target){
    if(chassis_mode != ChassisMode.CENTEROBJECT) return;
    double speed = -center_object_pid.calculate(_measure, _target);
    drive(0.3, speed, 0, false, false);
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
      if( (current_distance > _distance - error && current_distance < _distance + error)){
        m_frontLeft.resetDriveEncoder();
        chassis_mode = ChassisMode.MANUAL;
      }
    }while(chassis_mode == ChassisMode.DRIVEDISTANCE);
  }




  public void autoBalanceAll(){
    // double correction1 = autoBalancePitchRun();
    double correction2 = -autoBalanceRollRun();
    // drive(correction1, correction2, 0.0, false, false);
    drive(correction2, 0.0, 0.0, false, false);
  }
  // private PIDController balance_pid_pitch = new PIDController(.1, 0, 0);
  // public double autoBalancePitchRun(){
  //   double correction_speed = balance_pid_pitch.calculate(IMURobot.getPitch(), 0);
  //   return correction_speed;
  // }
  private PIDController balance_pid_roll = new PIDController(.098, 0, 0.0);
  public double autoBalanceRollRun(){
    double correction_speed = balance_pid_roll.calculate(IMURobot.getRoll(), 0);
    return correction_speed;
  }
  public void autoBalanceStop(){
    // balance_pid_pitch.reset();
    balance_pid_roll.reset();
    //stop for wheels is implicit in drive method controlled by the joystick
  }




  private PIDController compass_pid = new PIDController(0.05, 0.1, 0.03);
  public boolean setTurn(int desired_angle){
    double current_angle = IMURobot.getPitch();
    System.out.println("Current Angle: "+ current_angle+ "   desired_angle: "+desired_angle);
    int error = 1;
    double correction_speed = -compass_pid.calculate(current_angle, desired_angle);
    drive(0.0, 0.0, correction_speed, false, false);

    if(current_angle > desired_angle-error && current_angle <desired_angle+current_angle){
      compass_pid.reset();
      drive(0.0, 0.0, 0.0, false, false);

      return true;
    }
    else 
      return false;
  }


  
  public void stopModules(){
    m_frontLeft.stop();;
    m_frontRight.stop();;
    m_backLeft.stop();;
    m_backRight.stop();;  
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    m_frontLeft.setDesiredState(desiredStates[2], false);
    m_frontRight.setDesiredState(desiredStates[3], false);
    m_backLeft.setDesiredState(desiredStates[0], false);
    m_backRight.setDesiredState(desiredStates[1], false);  
  }

  public SwerveModuleNEO getDistanceReference(){
    return m_frontRight;
  } 


}
