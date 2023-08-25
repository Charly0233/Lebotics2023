// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.opencv.video.SparsePyrLKOpticalFlow;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenixpro.configs.CurrentLimitsConfigs;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IMUElevator;

public class ElevatorSubsystem extends SubsystemBase {
  //private static VictorSPX motor1 = new VictorSPX(11);
  //private static VictorSPX motor2 = new VictorSPX(12);
  private static final CANSparkMax motor1 = new CANSparkMax(10, MotorType.kBrushless);

  private RelativeEncoder motor1_encoder;
  
  PIDController pid = new PIDController(0.006, 0, 0.001); //p = 0.006, d = 0.001
  private static final double MOTOR_LIMIT = 110;
  
  static double motor_limit_rotation = MOTOR_LIMIT;
  private final double force_speed1 = 0.65;
  private final double force_speed2 = 0.65;

  public ElevatorSubsystem() {
    motor1_encoder = motor1.getEncoder();

  }
  
  public void setMotorLimit(){
    double pitchAngleDegrees = IMUElevator.getPitch(); // ahrs.getPitch();#
    motor_limit_rotation = pitchAngleDegrees+90;
  }
  public void resetMotorLimit(){
    double pitchAngleDegrees = IMUElevator.getPitch(); // ahrs.getPitch();#
    motor_limit_rotation = pitchAngleDegrees+90;
  }


  public void move(double speed){
    double pitchAngleDegrees = IMUElevator.getPitch(); // ahrs.getPitch();
    speed *= -1;
    speed *= force_speed2;
    if(speed >= 0.1 && pitchAngleDegrees <  motor_limit_rotation){ //ascending
      moveLeft(speed);
      moveRight(speed);
      System.out.println("ASCENDING");
    }
    else if(speed <= -0.1){//descending
      moveLeft(-1);
      moveRight(-1);
      System.out.println("ASCENDING");

    }
    else{
      moveLeft(0.0);
      moveRight(0.0);
    }


  }

  public void moveLeft(double speed){
    motor1.set( speed);
  }

  public void moveRight(double speed){
    motor1.set( speed);
  }

  public void stop(){
    motor1.set( 0);
  }

  public void moveMaster(double _lower , double _raise ){
    double pitchAngleDegrees = IMUElevator.getPitch(); // ahrs.getPitch();
    _lower *= force_speed1;
    _raise *= force_speed1;

    if(_raise >= 0.1  && pitchAngleDegrees <  motor_limit_rotation){ //ascending
      moveLeft(_raise);
      moveRight(_raise);
    } else if(_lower >= 0.1){
      moveLeft(-0.05);
      moveRight(-0.05);
    }else{
      //This is declared in the  move(double) Function

    }
  }

  // public void setWantedPosition(int _wanted_position){
  //   wanted_position = _wanted_position;
  // }

  public double getElevationAngle(){
    return IMUElevator.getPitch();
  }

  @Override
  public void periodic() {
    // double pitch = getElevationAngle();
    // double speed = -pid.calculate(pitch, wanted_position);
    // speed = (speed > 0.01) ? speed : 0.0;
    // System.out.println("pitch: " + pitch + " result: " + speed + " wanted: "+wanted_position);
    // SmartDashboard.putNumber("Elevator angle", pitch);
    // moveLeft(speed);
    // moveRight(speed);

    System.out.println("POSITION: "+motor1_encoder.getPosition());


  }
}
