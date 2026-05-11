package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class AgitateRollerTest extends Command {
    private ShooterSubsystem shooter;
    public AgitateRollerTest(ShooterSubsystem shooter){
        this.shooter = shooter;
        addRequirements(shooter);
        
    }
    @Override
    public void execute(){
        shooter.getRollerMotor().set(-0.6);
    }
    @Override
    public void end(boolean interrupted){
        shooter.getRollerMotor().set(0);
        
    }
    @Override
    public boolean isFinished() {
        return false; // keeps running until cancelled
    }
}
