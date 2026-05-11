package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.controls.PositionDutyCycle;

import frc.robot.subsystems.ShooterSubsystem;


public class ClimbCommand extends Command {
    private final TalonFX ClimbTalon;
    private final ShooterSubsystem shooter;

    public ClimbCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
        ClimbTalon = shooter.getClimbMotor();
    }

    @Override
    public void execute(){
    //if (ClimbTalon.getPosition().getValueAsDouble() >= -187){
            ClimbTalon.set(-0.67);
       //}
    //else{
      //ClimbTalon.set(0);
    //}
    }
    @Override
    public void end(boolean interrupted){ 
        ClimbTalon.set(0);
    }
    /*
    @Override
    public boolean isFinished() {
        return false; // keeps running until cancelled
    }*/
}
