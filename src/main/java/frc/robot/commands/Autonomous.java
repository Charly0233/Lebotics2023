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

          
new GrippingCommand(_gripperSubsystem,_elevatorSubsystem), //Jala
new WaitCommand(100), //Espera
//new DriveDistance(_chassisSwerveSubsystem, -35, InitializeWheels.VERTICAL), //Atras
new riseToShot(_elevatorSubsystem,78), //Eleva

new DriveDistance(_chassisSwerveSubsystem, 65, InitializeWheels.VERTICAL), //Delante

new WaitCommand(50), //Espera

new LaunchCommand(_gripperSubsystem), //Dispara

new WaitCommand(150), //Espera
new stopLaunchCommand(_gripperSubsystem), //Deja de disparar
new WaitCommand(80), //Espera
//new DriveDistance(_chassisSwerveSubsystem, -15, InitializeWheels.VERTICAL), //Atras
new riseToShot(_elevatorSubsystem,2), //Baja
//new WaitCommand(85),



///center



      new WaitCommand(30),
      new riseToShot(_elevatorSubsystem,5), //SUBE
      //new DriveDistance(_chassisSwerveSubsystem, -15, InitializeWheels.VERTICAL), //Atras
      //new DriveDistance(_chassisSwerveSubsystem, -15, InitializeWheels.VERTICAL), //Atras
      new DriveDistance(_chassisSwerveSubsystem, -58, InitializeWheels.VERTICAL), //Atras
      new DriveDistance(_chassisSwerveSubsystem, -13, InitializeWheels.VERTICAL), //Atras
      new riseToShot(_elevatorSubsystem,2), //baja
      //new DriveDistance(_chassisSwerveSubsystem, -13, InitializeWheels.VERTICAL), //Atras
      new DriveDistance(_chassisSwerveSubsystem, -104, InitializeWheels.VERTICAL), //Atras
      new DriveDistance(_chassisSwerveSubsystem, 25, InitializeWheels.VERTICAL), //Adelante
      new autoBalanceCommand(_chassisSwerveSubsystem)


      
      /*   new DriveDistance(_chassisSwerveSubsystem, -10, InitializeWheels.VERTICAL), //Atras
      new DriveDistance(_chassisSwerveSubsystem, -10, InitializeWheels.VERTICAL), //Atras
      new DriveDistance(_chassisSwerveSubsystem, -20, InitializeWheels.VERTICAL), //Atras/* */
      //new DriveDistance(_chassisSwerveSubsystem, -10, InitializeWheels.VERTICAL), //Atras
      //new DriveDistance(_chassisSwerveSubsystem, - 115, InitializeWheels.VERTICAL),
    
      
///right
/* 
new DriveDistance(_chassisSwerveSubsystem, 25, InitializeWheels.LATERAL),
new WaitCommand(500),
new DriveDistance(_chassisSwerveSubsystem, - 115, InitializeWheels.VERTICAL)
*/

///left
/* 
new WaitCommand(10),
new DriveDistance(_chassisSwerveSubsystem, - 105, InitializeWheels.VERTICAL)

 





    //prueba 1
      /* 
      new DriveDistance(_chassisSwerveSubsystem, 150,InitializeWheels.VERTICAL ),
      new DriveDistance(_chassisSwMerveSubsystem, 100,InitializeWheels.LATERAL ),
      new DriveDistance(_chassisSwerveSubsystem, 150,InitializeWheels.VERTICAL ),
      new DriveDistance(_chassisSwerveSubsystem, -100,InitializeWheels.LATERAL ),
      new DriveDistance(_chassisSwerveSubsystem, -150,InitializeWheels.VERTICAL ),
      */
      
      /* 
      new DriveDistance(_chassisSwerveSubsystem, 150, InitializeWheels.VERTICAL),
      new DriveDistance(_chassisSwerveSubsystem, 150, InitializeWheels.LATERAL),
      new DriveDistance(_chassisSwerveSubsystem, 100, InitializeWheels.VERTICAL),
      new DriveDistance(_chassisSwerveSubsystem, 300, InitializeWheels.LATERAL),
      */
/* 
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
*/

    );
  }
}
