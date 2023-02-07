// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ChassisMode;
import frc.robot.subsystems.ChassisSwerveSubsystem;
import frc.robot.subsystems.LimeLight2Subsystem;

public class LimeLight2Command extends CommandBase {
  LimeLight2Subsystem subsystem;
  ChassisSwerveSubsystem subsystem2;
  boolean turn_lights;
  public LimeLight2Command(LimeLight2Subsystem _subsystem, ChassisSwerveSubsystem _subsystem2) {
    subsystem = _subsystem;
    subsystem2 = _subsystem2;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    turn_lights = false;
  }

  @Override
  public void execute() {
    if(Constants.driver1.getRightStickButtonPressed()){
      turn_lights = !turn_lights;
    }




    if(turn_lights && subsystem.turnOnLight()){
      subsystem2.setChassisMode(ChassisMode.CENTEROBJECT);
      subsystem2.centerObjectLeftRight(subsystem.getTx(), 0.0);

    }else if(subsystem.turnOffLight()){
      subsystem2.setChassisMode(ChassisMode.MANUAL);


    }




  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
