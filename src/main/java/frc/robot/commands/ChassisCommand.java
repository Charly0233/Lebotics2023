// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.ControllerPS4;
import frc.robot.Constants.ControllerType;
import frc.robot.Constants.ControllerXbox;
import frc.robot.subsystems.ChassisTankSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ChassisCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ChassisTankSubsystem m_subsystem;
  private boolean auto_balance_on, auto_balance_off;

  public ChassisCommand(ChassisTankSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {


    double throttle = (Constants.driver1_type == ControllerType.PS4) ? -ControllerPS4.driver1.getLeftY() : -ControllerXbox.driver1.getLeftY();
    double throttle_turn = (Constants.driver1_type == ControllerType.PS4) ? ControllerPS4.driver1.getRightX() : ControllerXbox.driver1.getRightX();
    // boolean left = Constants.driver1.getLeftBumper();
    // boolean right = Constants.driver1.getRightBumper();
    boolean left = (Constants.driver1_type == ControllerType.PS4) ? ControllerPS4.driver1.getL1Button() : ControllerXbox.driver1.getLeftBumper();
    boolean right = (Constants.driver1_type == ControllerType.PS4) ? ControllerPS4.driver1.getR1Button() : ControllerXbox.driver1.getRightBumper();
    m_subsystem.drive(throttle, throttle_turn, left, right);


    switch((Constants.driver1_type == ControllerType.PS4) ? ControllerPS4.driver1.getPOV() : ControllerXbox.driver1.getPOV()){
      case 0:{
        m_subsystem.setTurn(0);
        break;
      }
      case 90:{
        m_subsystem.setTurn(90);
        break;
      }
      case 180:{
        m_subsystem.setTurn(180);
        break;
      }
      case 270:{
        m_subsystem.setTurn(-90);
        break;
      }
      default:{
        break;
      }
    }
    if(auto_balance_on) {
      m_subsystem.autoBalanceRun();
      SmartDashboard.putBoolean("AutoBalance", true);

    }else{
      m_subsystem.autoBalanceStop();
      SmartDashboard.putBoolean("AutoBalance", false);    }

    if ((Constants.driver1_type == ControllerType.PS4) ? ControllerPS4.driver1.getCircleButton() : ControllerXbox.driver1.getBButton()  == true) {
      if (!auto_balance_off) {
        auto_balance_on = !auto_balance_on;
        auto_balance_off = true;
      }
    }
    else {
      auto_balance_off = false;
    }

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
