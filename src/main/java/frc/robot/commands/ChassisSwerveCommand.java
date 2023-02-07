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
import frc.robot.Constants.ChassisMode;
import frc.robot.Constants.SwerveDriveChassis;
import frc.robot.subsystems.ChassisSwerveSubsystem;

public class ChassisSwerveCommand extends CommandBase {

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private final ChassisSwerveSubsystem subsystem;

  public ChassisSwerveCommand(ChassisSwerveSubsystem _subsystem) {
    subsystem = _subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    subsystem.initializeWheels();
  

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    boolean fieldRelative = false;
    final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(Constants.driver1.getLeftY(), 0.1)) * SwerveDriveChassis.kMaxSpeed;
    final var ySpeed =  m_yspeedLimiter.calculate(MathUtil.applyDeadband(Constants.driver1.getLeftX(), 0.1)) * SwerveDriveChassis.kMaxSpeed;
    // final var rot =       -m_rotLimiter.calculate(MathUtil.applyDeadband(Constants.driver1.getRawAxis(4), 0.1)) * SwerveDriveChassis.kMaxAngularSpeed;
    final var rot =       -m_rotLimiter.calculate(MathUtil.applyDeadband(Constants.driver1.getRightX(), 0.1)) * SwerveDriveChassis.kMaxAngularSpeed;

    subsystem.drive(xSpeed, ySpeed, rot, fieldRelative, false);


    if (Constants.driver1.getStartButtonPressed()){
      subsystem.initializeWheels();
    }
    if(Constants.driver1.getBButtonPressed() ) {
      subsystem.setChassisMode(ChassisMode.DRIVEDISTANCE);
      subsystem.driveDistanceInCm(100);
    }



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
