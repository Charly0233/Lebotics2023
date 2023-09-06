// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
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
  private double EncoderDesface;
  
  private static final double MOTOR_LIMIT = 79;
  
  static double motor_limit_rotation = MOTOR_LIMIT;
  private final double force_speed1 = 0.65;
  
  //pid
  private RelativeEncoder encoder;
  final double kP = 0.1; //velocidad
  final double kI = 0.5; //correcion
  final double kD = 0.3; //resta de velocidad
  final double ilimit = 1;

  double setpoint = 0;
  double errorSum = 0;
  double lasTimestamp = 0;
  double lastError = 0;
  //PIDController pid = new PIDController(0.006, 0, 0.001); //p = 0.006, d = 0.001
  
  /*public void MoveTo(double objective){
    setpoint=objective;
    double sensorPosition = encoder.getPosition();
    double error = setpoint - sensorPosition;
    double dt = Timer.getFPGATimestamp() - lasTimestamp;
    if(Math.abs(error)<ilimit){
      errorSum += error*dt;
    }

    double errorRate = (error-lastError) / dt;
    double outputSpeed = kP * error + kI *errorSum + kD * errorRate;
  
    moveLeft(outputSpeed);

    lasTimestamp = Timer.getFPGATimestamp();
    lastError = error;

    //return (outputSpeed==0) ? true : false;
  }*/

  public void MoveTo(double pos){
    double speedd = 0.5;
    double sensorPosition = encoder.getPosition();
    double goal = pos - sensorPosition;
    speedd=goal/100;

    if(sensorPosition < pos && sensorPosition < 100){
      moveLeft(speedd);
    }else if(sensorPosition > pos+2 && sensorPosition > 1   ){
      moveLeft(speedd);
    }
   // System.out.println("goal" + goal);

     /*if(goal<=1 ){
      //goal = 3;
      return true;
    } else return false;
  */}

  public ElevatorSubsystem() {
    encoder = motor1.getEncoder();
    EncoderDesface = encoder.getPosition();
    lasTimestamp=Timer.getFPGATimestamp();
    //MoveTo(0);
  }
  
  public void setMotorLimit(){
    double pitchAngleDegrees = IMUElevator.getPitch(); // ahrs.getPitch();#
    motor_limit_rotation = pitchAngleDegrees+90;
  }
  public void resetMotorLimit(){
    double pitchAngleDegrees = IMUElevator.getPitch(); // ahrs.getPitch();#
    motor_limit_rotation = pitchAngleDegrees+90;
  }

  public double getEncoderPosiiton(){
    return encoder.getPosition();
  }


  public void move(double speed){
    double pitchAngleDegrees = IMUElevator.getPitch(); // ahrs.getPitch();
    //System.out.print("algo:  "+ pitchAngleDegrees);
    speed *= -1;
    speed *= force_speed1;
    

    if(speed >= 0.1 && encoder.getPosition() < 80){ //ascending
      moveLeft(0.5);
      moveRight(0.5);
      //System.out.println("ASCENDING");
    }
    else if(speed <= -0.1){//descending
      moveLeft(-.3);
      moveRight(-.3);
      //System.out.println("ASCENDING");
      
    }else if(Math.abs(pitchAngleDegrees) > 23) {
      moveLeft(0.03);
      moveRight(0.03);
    }
    else {
      moveLeft(0);
      moveRight(0);
    }
  }

  public void noMove(){
    double pitchAngleDegrees = IMUElevator.getPitch(); // ahrs.getPitch();

    if(Math.abs(pitchAngleDegrees) > 23) {
      moveLeft(0.03);
      moveRight(0.03);
    }
    else {
      moveLeft(0);
      moveRight(0);
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

    

    if(_raise >= 0.1  /*&& Math.abs(pitchAngleDegrees) <  75*/ && encoder.getPosition() < 80){ //ascending
      moveLeft(0.5);
      moveRight(0.5);
    }
    else if( encoder.getPosition() > 1 && _lower >= 0.1 ){
      moveLeft(-0.3);
      moveRight(-0.3);
    }else if(Math.abs(pitchAngleDegrees) > 23){
      //This is declared in the  move(double) Function
      moveLeft(0.03);
      moveRight(0.03);
    }
    else {
    moveLeft(0);
    moveRight(0);

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

    System.out.println("POSITION: "+encoder.getPosition());
  }

  public void resetEncoder(){
    encoder.setPosition(0);
  }
}
