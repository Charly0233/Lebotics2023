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
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;

public class GripperSubsystem extends SubsystemBase {
 
  VictorSPX seat_motor = new VictorSPX(12);

  public GripperSubsystem() {
  
  }


  public void grab(double _input){
    if(_input > .1 || _input < -.1){
      seat_motor.set(ControlMode.PercentOutput, -_input);

    }else{
      seat_motor.set(ControlMode.PercentOutput, 0);
    }
  }

  public void grabTake(boolean _input){
    if(_input){
      seat_motor.set(ControlMode.PercentOutput, 0.4);
    }
  }
  
  public void grabRelease(boolean _input){
    if(_input){
      seat_motor.set(ControlMode.PercentOutput, -0.4);
    }
  }

		
  @Override
  public void periodic() {
  }



  
}
