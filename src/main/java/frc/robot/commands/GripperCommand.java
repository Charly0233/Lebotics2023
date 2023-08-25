// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ControllerPS4;
import frc.robot.Constants.ControllerType;
import frc.robot.Constants.ControllerXbox;
import frc.robot.subsystems.GripperSubsystem;

public class GripperCommand extends CommandBase {
  private final GripperSubsystem subsystem;

  public GripperCommand(GripperSubsystem _subsystem) {
    subsystem = _subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.grab( (Constants.driver2_type == ControllerType.PS4) ? ControllerPS4.driver2.getRightY() : ControllerXbox.driver2.getRightY());
    //diver1 and inverted bumpers
    subsystem.grabTake( (Constants.driver2_type == ControllerType.PS4) ? ControllerPS4.driver2.getSquareButton() : ControllerXbox.driver2.getXButton());
    subsystem.grabTake2( (Constants.driver2_type == ControllerType.PS4) ? ControllerPS4.driver2.getTriangleButton() : ControllerXbox.driver2.getYButton());
    subsystem.grabRelease( (Constants.driver2_type == ControllerType.PS4) ? ControllerPS4.driver2.getCircleButton() : ControllerXbox.driver2.getBButton());
    subsystem.grabRelease2( (Constants.driver2_type == ControllerType.PS4) ? ControllerPS4.driver2.getCrossButton() : ControllerXbox.driver2.getAButton());
    subsystem.grabReleaseSTRONG( (Constants.driver2_type == ControllerType.PS4) ? ControllerPS4.driver2.getR2Button() : ControllerXbox.driver2.getRightBumper());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
