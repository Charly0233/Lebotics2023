// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.InitializeWheels;
import frc.robot.subsystems.ChassisSwerveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Autonomous extends SequentialCommandGroup {
  /** Creates a new Autonomous. */
  public Autonomous(ChassisSwerveSubsystem _chassisSwerveSubsystem, ElevatorSubsystem _elevatorSubsystem, GripperSubsystem _gripperSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new DriveDistance(_chassisSwerveSubsystem, -25, InitializeWheels.LATERAL)

      new DriveDistance(_chassisSwerveSubsystem, -25, InitializeWheels.VERTICAL),
      new WaitCommand(1500),
      new ParallelRaceGroup(
        new RiseArm(_elevatorSubsystem, 5.6, -0.85),
        new WaitCommand(1500)
      ),
      new ParallelRaceGroup(
        new DriveDistance(_chassisSwerveSubsystem, 45,InitializeWheels.VERTICAL),
        new RiseArm(_elevatorSubsystem, 5.6, -0.85)
      ),
      new ParallelRaceGroup(
        new LeaveObject(_gripperSubsystem),
        new RiseArm(_elevatorSubsystem, 5.6, -0.85)
      ),
      new ParallelRaceGroup(
        new DriveDistance(_chassisSwerveSubsystem, -130,InitializeWheels.VERTICAL),
        new RiseArm(_elevatorSubsystem, 1, 0.03)
      ),
      new ParallelRaceGroup(
        new BalanceCommand(_chassisSwerveSubsystem),
        new WaitCommand(7000)
      ),
      new DriveTurn(_chassisSwerveSubsystem, 5)



    );
  }
}
