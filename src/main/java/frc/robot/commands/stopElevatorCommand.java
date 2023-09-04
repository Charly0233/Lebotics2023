package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class stopElevatorCommand extends CommandBase{
    private final ElevatorSubsystem subsystem;

    public stopElevatorCommand(ElevatorSubsystem subsystem) {
        this.subsystem = subsystem;
    }
    
    
    @Override
    public void execute(){
        subsystem.moveLeft(0);
    }


    
    @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
    
}
