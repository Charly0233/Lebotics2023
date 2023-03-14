// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;

public class LeaveObject extends CommandBase {
  private final GripperSubsystem subsystem;
  private static boolean terminate;

  /** Creates a new GrabObject. */
  public LeaveObject(GripperSubsystem _subsystem) {
    subsystem = _subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    terminate = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    for(int i = 0; i<4000; i++){
      subsystem.grab(0.85);
    }
    terminate = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.grab(0.0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return terminate;
  }
}
