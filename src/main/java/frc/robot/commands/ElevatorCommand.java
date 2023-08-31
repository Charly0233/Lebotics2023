// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ControllerPS4;
import frc.robot.Constants.ControllerType;
import frc.robot.Constants.ControllerXbox;
import frc.robot.subsystems.ElevatorSubsystem;


public class ElevatorCommand extends CommandBase {
private final ElevatorSubsystem subsystem;
private Direction direction;
boolean isDone = false;

  public ElevatorCommand(ElevatorSubsystem _subsystem) {
    subsystem = _subsystem;
    addRequirements(subsystem);
    direction=Direction.Off;
  }

  public static enum Direction {
    Off(0),UP(1), MID(2), DOWN(3);

    int direction;

    private Direction(int direction) {
        this.direction = direction;
    }
}


  @Override
  public void initialize() {}


  @Override
  public void execute() {
    System.out.print("isDone "+isDone);
    /*if(isDone==true){
      direction=Direction.Off;
      isDone=false;
    }else*/
     if((Constants.driver2_type == ControllerType.PS4) ? ControllerPS4.driver2.getL1ButtonPressed() : ControllerXbox.driver2.getLeftBumperPressed()){
      subsystem.MoveTo(82);
    } else if((Constants.driver2_type == ControllerType.PS4) ? ControllerPS4.driver2.getShareButtonPressed() : ControllerXbox.driver2.getBackButtonPressed()) {
      //direction=Direction.MID;
      subsystem.MoveTo(74);
    } else if((Constants.driver2_type == ControllerType.PS4) ? ControllerPS4.driver2.getL3ButtonPressed() : ControllerXbox.driver2.getLeftStickButtonPressed()){
      subsystem.MoveTo(74);
      //direction=Direction.DOWN;
    } else {
      subsystem.moveMaster((Constants.driver2_type == ControllerType.PS4) ? ControllerPS4.driver2.getL2Axis() : ControllerXbox.driver2.getLeftTriggerAxis(),
                             (Constants.driver2_type == ControllerType.PS4) ? ControllerPS4.driver2.getR2Axis() : ControllerXbox.driver2.getRightTriggerAxis());
    }

    
    
    // subsystem.move((Constants.driver2_type == ControllerType.PS4) ? ControllerPS4.driver2.getLeftY() : ControllerXbox.driver2.getLeftY() );

   // System.out.println("direction: "+direction);
  /* 
    switch (direction) {
      case UP:
        isDone = subsystem.MoveTo(82);
        break;
      case MID:
      isDone = subsystem.MoveTo(74);
        break;
      case DOWN:
      isDone = subsystem.MoveTo(58);
        break;
      default:
        subsystem.moveMaster((Constants.driver2_type == ControllerType.PS4) ? ControllerPS4.driver2.getL2Axis() : ControllerXbox.driver2.getLeftTriggerAxis(),
                             (Constants.driver2_type == ControllerType.PS4) ? ControllerPS4.driver2.getR2Axis() : ControllerXbox.driver2.getRightTriggerAxis());

        break;
    }

*/

  }


  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}
