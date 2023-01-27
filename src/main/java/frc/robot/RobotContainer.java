// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ChassisCommand;
import frc.robot.commands.ChassisSwerveCommand;
import frc.robot.commands.GripperCommand;
import frc.robot.subsystems.ChassisTankSubsystem;
import frc.robot.subsystems.ChassisSwerveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  final ChassisTankSubsystem chassisSubsystem = new ChassisTankSubsystem();
  final ChassisSwerveSubsystem chassisSwervesSubsystem = new ChassisSwerveSubsystem();
  
  private final GripperSubsystem gripperSubsystem = new GripperSubsystem();
  private final GripperCommand gripperCommand = new GripperCommand(gripperSubsystem);
 

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return null;
  }

  public Command getChassisCommand(String type){
    switch(type){
      case "tank":{
        final ChassisCommand chassisCommand = new ChassisCommand(chassisSubsystem);
        return chassisCommand;
      }
      case "swerve":{
        final ChassisSwerveCommand chassisCommand = new ChassisSwerveCommand(chassisSwervesSubsystem);
        return chassisCommand;
      }
      default:{
        final ChassisCommand chassisCommand = new ChassisCommand(chassisSubsystem);
        return chassisCommand;
      }
    }
  }
  public Command getGripperCommand(){
    return gripperCommand;
  }
}
