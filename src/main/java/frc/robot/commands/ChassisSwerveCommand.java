// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.SwerveModuleNEO;
import frc.robot.Constants.ChassisMode;
import frc.robot.Constants.ControllerPS4;
import frc.robot.Constants.ControllerType;
import frc.robot.Constants.ControllerXbox;
import frc.robot.Constants.InitializeWheels;
import frc.robot.Constants.SwerveDriveChassis;
import frc.robot.subsystems.ChassisSwerveSubsystem;

public class ChassisSwerveCommand extends CommandBase {

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  double kSensibility = 1.415;

  private final ChassisSwerveSubsystem subsystem;
  private boolean auto_balance = false;
  public ChassisSwerveCommand(ChassisSwerveSubsystem _subsystem) {
    subsystem = _subsystem;
    addRequirements(subsystem);
  }


  @Override
  public void initialize() {
    // subsystem.initializeWheels();
  

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    boolean fieldRelative = false;
    //deadzone changed from 0.1 to 0.15
    
    //final var xSpeed = -Math.pow(m_xspeedLimiter.calculate(MathUtil.applyDeadband((Constants.driver1_type == ControllerType.PS4) ? ControllerPS4.driver1.getLeftY() : ControllerXbox.driver1.getLeftY(), 0.15), kSensibility)) * SwerveDriveChassis.kMaxSpeed;
    //final var ySpeed =  Math.pow(m_yspeedLimiter.calculate(MathUtil.applyDeadband((Constants.driver1_type == ControllerType.PS4) ? ControllerPS4.driver1.getLeftX() : ControllerXbox.driver1.getLeftX(), 0.15), kSensibility))* SwerveDriveChassis.kMaxSpeed;
    final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband((Constants.driver1_type == ControllerType.PS4) ? ControllerPS4.driver1.getLeftY() : ControllerXbox.driver1.getLeftY(), 0.1)) * SwerveDriveChassis.kMaxSpeed;
    final var ySpeed =  m_yspeedLimiter.calculate(MathUtil.applyDeadband((Constants.driver1_type == ControllerType.PS4) ? ControllerPS4.driver1.getLeftX() : ControllerXbox.driver1.getLeftX(), 0.1)) * SwerveDriveChassis.kMaxSpeed;
    // final var rot =       -m_rotLimiter.calculate(MathUtil.applyDeadband(Constants.driver1.getRawAxis(4), 0.1)) * SwerveDriveChassis.kMaxAngularSpeed;
    
    //final var rot =       -Math.pow(m_rotLimiter.calculate(MathUtil.applyDeadband((Constants.driver1_type == ControllerType.PS4) ? ControllerPS4.driver1.getRightX() : ControllerXbox.driver1.getRightX(), 0.15), kSensibility)) * SwerveDriveChassis.kMaxAngularSpeed;
    final var rot =       -m_rotLimiter.calculate(MathUtil.applyDeadband((Constants.driver1_type == ControllerType.PS4) ? ControllerPS4.driver1.getRightX() : ControllerXbox.driver1.getRightX(), 0.1)) * SwerveDriveChassis.kMaxAngularSpeed;


    subsystem.drive(xSpeed, ySpeed, rot, fieldRelative, false);

    if ((Constants.driver1_type == ControllerType.PS4) ? ControllerPS4.driver1.getOptionsButtonPressed() : ControllerXbox.driver1.getStartButtonPressed()){
      subsystem.initializeWheels(InitializeWheels.VERTICAL);
    }
    if((Constants.driver1_type == ControllerType.PS4) ? ControllerPS4.driver1.getCircleButtonPressed() : ControllerXbox.driver1.getBButtonPressed() ) {
      subsystem.setChassisMode(ChassisMode.DRIVEDISTANCE);
      subsystem.driveDistanceInCm(100);
    }
    subsystem.enableTurbo( (Constants.driver1_type == ControllerType.PS4) ? ControllerPS4.driver1.getR3ButtonPressed() : ControllerXbox.driver1.getLeftStickButtonPressed() );


    
    if((Constants.driver1_type == ControllerType.PS4) ? ControllerPS4.driver1.getTriangleButtonPressed() : ControllerXbox.driver1.getYButtonPressed() ) {
      auto_balance = !auto_balance;
    }
    if(auto_balance){
      subsystem.autoBalanceAll();
      SmartDashboard.putBoolean("AutoBalance", true);
    }else{
      subsystem.autoBalanceStop();
      SmartDashboard.putBoolean("AutoBalance", false);
    }

    switch((Constants.driver1_type == ControllerType.PS4) ? ControllerPS4.driver1.getPOV() : ControllerXbox.driver1.getPOV()){
      case 0:{
        subsystem.setTurn(0);
        break;
      }
      case 90:{
        subsystem.setTurn(90);
        break;
      }
      case 180:{
        subsystem.setTurn(180);
        break;
      }
      case 270:{
        subsystem.setTurn(-90);
        break;
      }
      default:{
        break;
      }
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
