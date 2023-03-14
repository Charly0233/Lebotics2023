// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ControllerPS4;
import frc.robot.Constants.ControllerType;
import frc.robot.Constants.ControllerXbox;
import frc.robot.subsystems.ElevatorSubsystem;


public class ElevatorCommand extends CommandBase {
private final ElevatorSubsystem subsystem;

  public ElevatorCommand(ElevatorSubsystem _subsystem) {
    subsystem = _subsystem;
    addRequirements(subsystem);
  }


  @Override
  public void initialize() {}


  @Override
  public void execute() {
    subsystem.move((Constants.driver2_type == ControllerType.PS4) ? ControllerPS4.driver2.getLeftY() : ControllerXbox.driver2.getLeftY() );

    // subsystem.move((Constants.driver2_type == ControllerType.PS4) ? ControllerPS4.driver2.getTriangleButtonPressed() : ControllerXbox.driver2.getYButtonPressed(),
                  //  (Constants.driver2_type == ControllerType.PS4) ? ControllerPS4.driver2.getCrossButtonPressed() : ControllerXbox.driver2.getAButtonPressed());
    
    subsystem.moveMaster((Constants.driver1_type == ControllerType.PS4) ? ControllerPS4.driver1.getL2Axis() : ControllerXbox.driver1.getLeftTriggerAxis(),
                         (Constants.driver1_type == ControllerType.PS4) ? ControllerPS4.driver1.getR2Axis() : ControllerXbox.driver1.getRightTriggerAxis() );
  
  
  }


  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}
