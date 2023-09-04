package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;

public class GrippingCommand extends CommandBase {
    private final GripperSubsystem subsystem;
    public GrippingCommand(GripperSubsystem _subsystem, ElevatorSubsystem sub) {
        subsystem=_subsystem;
        addRequirements(subsystem,sub);
    }

    @Override
    public void execute(){
        subsystem.chupa(true);
        System.out.println(
            "chupo"
        );
    }

    public void lanza(){
        subsystem.lanza(true);
    }
    
    // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
