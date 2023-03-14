// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.Date;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitCommand extends CommandBase {


  /** Creates a new WaitCommand. */
  public WaitCommand(long _target_time) {
    target_time = _target_time;

  }
  Date date;
  long reference_time;
  long target_time;
  boolean terminate;
  @Override
  public void initialize() {
    date = new Date();
    reference_time = date.getTime();
    terminate = false;
    // System.out.println("Timer STARTED");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Date date2 = new Date();
    long current_time = date2.getTime();

    current_time = current_time - reference_time;
    // System.out.println(current_time);
    if(current_time > target_time){
      terminate = true;
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // System.out.println("Timer ENDED");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return terminate;
  }
}
