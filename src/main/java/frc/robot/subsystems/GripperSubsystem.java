// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;

public class GripperSubsystem extends SubsystemBase {
 private static final CANSparkMax motor1 = new CANSparkMax(12, MotorType.kBrushless);
 private static final CANSparkMax motor2 = new CANSparkMax(13, MotorType.kBrushless);

/*private RelativeEncoder motor1_encoder;
 private RelativeEncoder motor2_encoder;
*/
  //VictorSPX seat_motor = new VictorSPX(12);

  public GripperSubsystem() {/* 
    motor1_encoder=motor1.getEncoder();
    motor2_encoder=motor2.getEncoder();*/
  
  }


  public void grab(double _input){
    if(_input > .1 ){
      motor1.set(_input);
      motor2.set(_input*-1);
    }else if (_input < -.10){
      motor1.set(_input);
      motor2.set(_input*-1);
    }else{
      motor1.set(0);
      motor2.set(0);
    }
  }

  public void chupa(boolean _input){
    if(_input){
      motor1.set(0.3);
      motor2.set(-0.3);
    }
  }
  
  public void muerde(boolean _input){
    if(_input){
      motor1.set(0.7);
      motor2.set(-0.7);
    }
  }
  
  public void escupe(boolean _input){
    if(_input){ //Escupe
      motor1.set(-0.13);
      motor2.set(0.13);
    }
  }
  
  public void lanza(boolean _input){
    if(_input){ //Lanza
      motor1.set(-0.5);
      motor2.set(0.5);
    }
  }

  public void dispara(boolean _input){
    if(_input){ //Dispara
      motor1.set(-1);
      motor2.set(1);
    }
  }

  public boolean deten(){
    motor1.set(0);
    motor2.set(0);
    return true;
  }
		
  @Override
  public void periodic() {
  }



  
}
