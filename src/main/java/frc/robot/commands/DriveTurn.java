// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IMURobot;
import frc.robot.subsystems.ChassisSwerveSubsystem;

public class DriveTurn extends CommandBase {
  private final ChassisSwerveSubsystem subsystem;
  private  boolean terminate;
  private int angle;
  /** Creates a new DriveTurn. */
  public DriveTurn(ChassisSwerveSubsystem _subsystem, int _angle) {
    subsystem = _subsystem;
    addRequirements(subsystem);
    angle = _angle;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    terminate = false;
    IMURobot.reset();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean turn_done = false;
    turn_done = subsystem.setTurn(angle);
    System.out.println("Turning to: "+angle+"°"+" -> "+IMURobot.getPitch());

    if(turn_done){
      terminate = true;
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return terminate;
  }
}
