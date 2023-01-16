// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;  
import edu.wpi.first.wpilibj.Encoder;
 
public class GripperSubsystem extends SubsystemBase {
  VictorSP angular_motor = new VictorSP(3);
  PIDController angular_pid = new PIDController(.1, 0, 0);
  private boolean angular_pid_restarted;
  private static final double cpr = 28; 

  Encoder enc = new Encoder(0,1);

  public GripperSubsystem() {
    enc.setDistancePerPulse(1.8/7); //Distance of X every Y pulses (X/Y)
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
  @Override
  public void periodic() {

    // System.out.println(enc.getDistance()*360);
    System.out.println(enc.getDistance());

    // angular_pid.calculate(imu.getGyroAngleX(), 0);
  }



  
}
