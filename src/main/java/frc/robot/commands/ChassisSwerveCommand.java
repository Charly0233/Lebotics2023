// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.SwerveModuleNEO;
import frc.robot.Constants.SwerveDriveTrainChassis;
import frc.robot.subsystems.ChassisSwerveSubsystem;

public class ChassisSwerveCommand extends CommandBase {

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private final ChassisSwerveSubsystem subsystem;

  /** Creates a new ChassisSwerveCommand. */
  public ChassisSwerveCommand(ChassisSwerveSubsystem _subsystem) {
    subsystem = _subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    boolean fieldRelative = false;
    final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(Constants.driver1.getLeftY(), 0.1)) * SwerveDriveTrainChassis.kMaxSpeed;
    final var ySpeed =  m_yspeedLimiter.calculate(MathUtil.applyDeadband(Constants.driver1.getLeftX(), 0.1)) * SwerveDriveTrainChassis.kMaxSpeed;
    final var rot =       -m_rotLimiter.calculate(MathUtil.applyDeadband(Constants.driver1.getRightX(), 0.1)) * SwerveDriveTrainChassis.kMaxAngularSpeed;

    subsystem.drive(xSpeed, ySpeed, rot, fieldRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
