package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;

public class LaunchCommand extends CommandBase {
    private final GripperSubsystem subsystem;

    public LaunchCommand(GripperSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute(){
        subsystem.lanza(true);
        System.out.println(
            "termine de lanzar"
        );
        try {
            java.util.concurrent.TimeUnit.MILLISECONDS.sleep(50);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        //subsystem.lanza(false);
    }


    
    @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
