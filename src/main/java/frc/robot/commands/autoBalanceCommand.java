package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IMUElevator;
import frc.robot.subsystems.ChassisSwerveSubsystem;

public class autoBalanceCommand extends CommandBase {
    private final ChassisSwerveSubsystem subsystem;
    static final double kOffBalanceAngleThresholdDegrees = 10;
   // AHRS ahrs;
    double speedX;

    public autoBalanceCommand(ChassisSwerveSubsystem _subsystem){
        subsystem = _subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        subsystem.resetEncoders();
    }

    @Override 
    public void execute(){
        double pitchAngleDegrees = IMUElevator.getPitch(); // ahrs.getPitch();
        SmartDashboard.putNumber("angles", pitchAngleDegrees);
        if(Math.abs(pitchAngleDegrees)>= Math.abs(kOffBalanceAngleThresholdDegrees)){
            double pitchAngleRadians = pitchAngleDegrees * (Math.PI/180.0);
            speedX = Math.sin(pitchAngleRadians) *-1;
        }
        else{
            speedX=0;
        }
        subsystem.drive(speedX, 0, 0, false, false);
    }

    @Override
    public void end(boolean interrupted){
        subsystem.resetEncoders();
    }
    
    @Override
    public boolean isFinished() {
        subsystem.resetEncoders();
        return false;
    }
}