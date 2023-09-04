package frc.robot.commands;

import java.util.concurrent.PriorityBlockingQueue;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class riseToShot extends CommandBase {
    private final ElevatorSubsystem subsystem;
  private  boolean terminate = false;
  private int x=70;
  private double EncoderDesface;
  

 // private static final CANSparkMax motor1 = new CANSparkMax(10, MotorType.kBrushless);
  //private RelativeEncoder encoder;


    public riseToShot(ElevatorSubsystem subsystem) {
        this.subsystem = subsystem;
        EncoderDesface = subsystem.getEncoderPosiiton();

        addRequirements(subsystem);
    }

    @Override
    public void execute(){
        double speedd = 0.5;
        double sensorPosition = subsystem.getEncoderPosiiton();
        double goal = x - sensorPosition;
        speedd=goal/100;

        if(sensorPosition < x+EncoderDesface && sensorPosition < 80){
            subsystem.moveLeft(speedd);

        }else if(sensorPosition > x+2+EncoderDesface && sensorPosition > 1   ){
            subsystem.moveLeft(speedd);
        }
        System.out.print("goal" + goal+"  ");

        if(goal<5){
            System.out.println(
                "termine de subir"
            );
        terminate=true;}
    }

    @Override
  public void end(boolean interrupted) {
    subsystem.moveLeft(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return terminate;
  }
    
}
