// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class RiseArm extends CommandBase {
  private final ElevatorSubsystem subsystem;
  private double position, speed;

  /** Creates a new RaiseArm. */
  public RiseArm(ElevatorSubsystem _subsystem, double _position, double _speed) {
    subsystem = _subsystem;
    addRequirements(subsystem);
    position = _position;
    speed = _speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.setMotorLimit(position);
    subsystem.move(speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.resetMotorLimit();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
