// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IMU;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.SPI;

public class ChassisTankSubsystem extends SubsystemBase {


  VictorSP chassis_motor_left = new VictorSP(0);
  VictorSP chassis_motor_right = new VictorSP(1);
  DifferentialDrive drive = new DifferentialDrive(chassis_motor_left, chassis_motor_right);
  private PIDController balance_pid = new PIDController(.1, 0, 0);
  private PIDController compass_pid = new PIDController(.015, 0, 0);

  private double speed, speedV;

  public ChassisTankSubsystem() {
    speed = 1;
    speedV = .4;
    chassis_motor_right.setInverted(true);
  }

  public CommandBase exampleMethodCommand() {
    return runOnce(
        () -> {

        });
  }

  // public void drive(double left_speed, double right_speed){
  //   if( (left_speed >= .1 || left_speed <= -.1) && (right_speed >= .1 || right_speed <= -.1)){
  //     chassis_motor_left1.set(ControlMode.PercentOutput ,left_speed);
  //     chassis_motor_left2.set(ControlMode.PercentOutput, left_speed);
  //     chassis_motor_right1.set(ControlMode.PercentOutput ,-right_speed);
  //     chassis_motor_right2.set(ControlMode.PercentOutput ,-right_speed);
  //   }else{
  //     chassis_motor_left1.set(ControlMode.PercentOutput ,0);
  //     chassis_motor_left2.set(ControlMode.PercentOutput, 0);
  //     chassis_motor_right1.set(ControlMode.PercentOutput ,0);
  //     chassis_motor_right2.set(ControlMode.PercentOutput ,0);
  //   }
  // }
  public void drive(double throttle1, double throttle_turn, boolean left, boolean right) {
    throttle1*=.85;
    throttle_turn*=.9;
		if (throttle1 >= .1) {
			if (throttle_turn >= 0.1) {
        // drive((throttle1 * speed), (throttle1 * speed) - (throttle_turn * speedV));
        drive.tankDrive((throttle1 * speed), (throttle1 * speed) - (throttle_turn * speedV));
			} else if (throttle_turn <= -0.1) {
        // drive((throttle1 * speed) - (-throttle_turn * speedV), (throttle1 * speed));
        drive.tankDrive((throttle1 * speed) - (-throttle_turn * speedV), (throttle1 * speed));
			} else {
        // drive( throttle1 * speed , throttle1 * speed);
        drive.tankDrive( throttle1 * speed , throttle1 * speed);
			}
		} else if (throttle1 <= -.1) {
			if (throttle_turn >= 0.1) {
        // drive( (throttle1 * speed), (throttle1 * speed) - (-throttle_turn * speedV));
        drive.tankDrive( (throttle1 * speed), (throttle1 * speed) - (-throttle_turn * speedV));
			} else if (throttle_turn <= -0.1) {
        // drive( (throttle1 * speed) - (throttle_turn * speedV), (throttle1 * speed));
        drive.tankDrive( (throttle1 * speed) - (throttle_turn * speedV), (throttle1 * speed));
			} else {
        // drive( throttle1 * speed, throttle1 * speed);
        drive.tankDrive( throttle1 * speed, throttle1 * speed);
			}
		} 
    else if(left){
      // drive(-.6,.6);
      drive.tankDrive(-.6,.6);
    }else if(right){
      // drive(.6,-.6);
      drive.tankDrive(.6,-.6);
    }else {
        // drive( 0, 0 );
        drive.tankDrive( 0, 0 );
		}

	} 

  public void autoBalanceRun(){
    double correction_speed = balance_pid.calculate(IMU.getIMU().getGyroAngleX(), 0);
    drive.tankDrive( correction_speed, correction_speed);

  }
  public void autoBalanceStop(){
    balance_pid.reset();
    //stop for wheels is implicit in drive method controlled by the joystick
  }

  public boolean setTurn(int desired_angle){
    double current_angle = IMU.getIMU().getGyroAngleZ();
    int error = 1;
    double correction_speed = compass_pid.calculate(current_angle, desired_angle);
    drive.tankDrive( correction_speed, -correction_speed);

    if(current_angle > desired_angle-error && current_angle <desired_angle+current_angle){
      compass_pid.reset();
      return true;
    }
    else 
      return false;
  }

  @Override
  public void periodic() {
    // System.out.println("angleX:"+imu.getGyroAngleX()+" angleY:"+imu.getGyroAngleY()+" angleZ:"+imu.getGyroAngleZ());
    SmartDashboard.putNumber("Angle X", IMU.getIMU().getGyroAngleX());
    SmartDashboard.putNumber("Angle Z", IMU.getIMU().getGyroAngleZ());
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
