package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;

public class stopLaunchCommand extends CommandBase{
    private final GripperSubsystem subsystem;
    private boolean terminate;

    public stopLaunchCommand(GripperSubsystem subsystem) {
        this.subsystem = subsystem;
        terminate=false;
    }
    
    
    @Override
    public void execute(){
        subsystem.deten();
        terminate=true;
    }


    
    @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return terminate;
  }
    
}
