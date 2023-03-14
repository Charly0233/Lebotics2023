// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.SwerveDriveChassis;
import frc.robot.commands.Autonomous;
import frc.robot.commands.ChassisCommand;
import frc.robot.commands.ChassisSwerveCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.GripperCommand;
import frc.robot.commands.LimeLight2Command;
import frc.robot.subsystems.ChassisTankSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ChassisSwerveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.LimeLight2Subsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

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
 
  private final LimeLight2Subsystem limeLight2Subsystem = new LimeLight2Subsystem();
  private final LimeLight2Command limeLight2Command = new LimeLight2Command(limeLight2Subsystem, chassisSwervesSubsystem);

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ElevatorCommand elevatorCommand = new ElevatorCommand(elevatorSubsystem);


  private final Autonomous autonomous = new Autonomous(chassisSwervesSubsystem, elevatorSubsystem, gripperSubsystem);
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}

  public ChassisSwerveSubsystem getChassisSwerveSubsystem(){
    return chassisSwervesSubsystem;
  }
  public Command getAutonomousCommand() {
    return autonomous;
  }
  // public Command getAutonomousCommand() {
  //   TrajectoryConfig trajectory_config = new TrajectoryConfig(
  //     AutonomousConstants.kMaxSpeedMetersPerSecond,
  //     AutonomousConstants.kMaxAccelerationMetersPerSecondSquared)
  //   .setKinematics(SwerveDriveChassis.m_kinematics);

  //   Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
  //     new Pose2d(0,0, new Rotation2d(0)),
  //     List.of(
  //       new Translation2d(1, 0)

  //     ),
  //     new Pose2d(2, 0, Rotation2d.fromDegrees(0)),
  //     trajectory_config
  //   );
  //   PIDController xController = new PIDController(1, 0, 0);
  //   PIDController yController = new PIDController(1, 0, 0);
    
  //   ProfiledPIDController thetaController = new ProfiledPIDController(1, 0, 0, AutonomousConstants.kThetaControllerConstraints);
  //   thetaController.enableContinuousInput(-Math.PI, Math.PI);

  //   SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
  //     trajectory,
  //     chassisSwervesSubsystem::getPose,
  //     SwerveDriveChassis.m_kinematics,
  //     xController,
  //     yController,
  //     thetaController,
  //     chassisSwervesSubsystem::setModuleStates,
  //     chassisSwervesSubsystem
    
  //   );
  //   return new SequentialCommandGroup(
  //     new InstantCommand( ()-> chassisSwervesSubsystem.resetOdometry(trajectory.getInitialPose())),
  //     swerveControllerCommand,
  //     new InstantCommand( ()-> chassisSwervesSubsystem.stopModules())
  //   );
  
  // }

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

  public Command getLimeLight2Command(){
    return limeLight2Command;
  }

  public Command getElevatorCommand(){
    return elevatorCommand;
  }
}
