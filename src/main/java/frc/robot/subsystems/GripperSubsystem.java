// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
 
public class GripperSubsystem extends SubsystemBase {
  VictorSP angular_motor = new VictorSP(3);
  PIDController angular_pid = new PIDController(.1, 0, 0);
  private boolean angular_pid_restarted;
  private final Servo gripper;
  Encoder enc = new Encoder(0,1);

  final int MIN_ANGLE = 100;
  final int MAX_ANGLE = 170;
  int gripper_angle;
  CANSparkMax m_left, m_right;

  Counter motorCounter = new Counter(new DigitalInput(7));
  VictorSPX seat_motor = new VictorSPX(12);

  public GripperSubsystem() {
    enc.setDistancePerPulse(1.8/7); //Distance of X every Y pulses (X/Y)
    gripper = new Servo(9);
    gripper_angle = MAX_ANGLE;

    m_left = new CANSparkMax(10,  MotorType.kBrushless);
    m_right = new CANSparkMax(11,  MotorType.kBrushless);
    motorCounter.reset();
  }

  public void grab(boolean _input1, boolean _input2){
    if( _input1 ){
      gripper_angle++;
    }else if( _input2 ){
      gripper_angle--;
    }
    gripper.setAngle(gripper_angle);
    // System.out.println("Gripper angle: "+gripper.getAngle()+ " logic: "+gripper_angle);
  }

  public void grab(double _input){
    if(_input > .1 || _input < -.1){
      m_left.set(_input);
      m_right.set(-_input);
      seat_motor.set(ControlMode.PercentOutput, _input);

    }else{
      m_left.set(0);
      m_right.set(0);
      seat_motor.set(ControlMode.PercentOutput, 0);
    }
  }


  public void rotateGripper(double _input){
    double error = 2;
    if(_input >= .1){
      //joystick - downward position
      if(enc.getDistance() >= 0-error){
        double move_speed = -angular_pid.calculate(enc.getDistance(), 90);
        angular_motor.set(move_speed);
        angular_pid_restarted = false;
      }
      else{
        angular_motor.set(0);
        if(!angular_pid_restarted){
          angular_pid.reset();
          angular_pid_restarted = true;
        }
      }
    }
    else if(_input <= -.1){
      //joystick - upward position
      if(enc.getDistance() <= 90 + error){
        double move_speed = -angular_pid.calculate(enc.getDistance(), 0);
        angular_motor.set(move_speed);
        angular_pid_restarted = false;

      }
      else{
        angular_motor.set(0);
        if(!angular_pid_restarted){
          angular_pid.reset();
          angular_pid_restarted = true;
        }      
      }
    }
    else{
      angular_motor.set(0);
      if(!angular_pid_restarted){
        angular_pid.reset();
        angular_pid_restarted = true;
      }
    }


		
  }

  int position = 0;
  @Override
  public void periodic() {
    position += motorCounter.get();
    System.out.println("Seat motor distance0: "+position);
    // System.out.println("Seat motor distance1: "+motorCounter.get());
    // System.out.println("Seat motor distance2: "+motorCounter.getDistance());
    // System.out.println("Seat motor distance3: "+motorCounter.getRate());
    motorCounter.reset();
    // System.out.println(enc.getDistance());
  }



  
}
