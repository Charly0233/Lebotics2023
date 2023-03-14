// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.InitializeWheels;
import frc.robot.subsystems.ChassisSwerveSubsystem;

public class DriveDistance extends CommandBase {
  private final ChassisSwerveSubsystem subsystem;
  private  boolean terminate;
  private double distance;
  private InitializeWheels wheels_mode;
  /** Creates a new DriveDistance. */
  public DriveDistance(ChassisSwerveSubsystem _subsystem, double _distance, InitializeWheels _mode) {
    subsystem = _subsystem;
    addRequirements(subsystem);
    distance = _distance;
    wheels_mode = _mode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    terminate = false;
    subsystem.getDistanceReference().resetDriveEncoder();
    subsystem.initializeWheels(wheels_mode);

  }
  PIDController drive_distance_pid = new PIDController(0.02, 0.1, 0.01);

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double error = 2;

    double current_distance = subsystem.getDistanceReference().getDriveDistance();
    double speed = drive_distance_pid.calculate(current_distance, distance);
    if(wheels_mode == InitializeWheels.VERTICAL){
      subsystem.drive(speed, 0, 0, false, false);
    }
    else if(wheels_mode == InitializeWheels.LATERAL){
      subsystem.drive(0, speed, 0, false, false);
    }
    System.out.println("Driving to: "+distance+"cm -> "+current_distance);
    if( (current_distance > Math.abs(distance) - error && current_distance < Math.abs(distance) + error)){
      subsystem.getDistanceReference().resetDriveEncoder();
      subsystem.drive(0, 0, 0, false, false);
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
