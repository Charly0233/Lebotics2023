// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
  * ledMode
  * 0 -> use the LED Mode set in the current pipeline
  * 1 -> force off
  * 2 -> force blink
  * 3 -> force on
*/
/*
 * camMode
 * 0 -> Vision processor
 * 1 -> Driver Camera (Increases exposure, disables vision processing)
 */
package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight2Subsystem extends SubsystemBase {

  /** Creates a new LimeLight2. */
  public LimeLight2Subsystem() {

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);

  }




  public boolean turnOnLight(){

    if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").getNumber(9).equals(1)){
      return true;
    }
    try{
      SmartDashboard.putBoolean("Light", true);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
      return true;
    }catch(Exception e){
      return false;
    }
  
  }
  public boolean turnOffLight(){
    if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").getNumber(9).equals(0)){
      return true;
    }
    try{
      SmartDashboard.putBoolean("Light", false);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
      return true;
    }catch(Exception e){
      return false;
    }
  }

  public double getTx(){
      return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
  }

  public double getTy(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
}



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
