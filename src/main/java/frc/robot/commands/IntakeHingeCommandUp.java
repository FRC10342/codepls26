package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.controls.PositionDutyCycle;

import frc.robot.subsystems.ShooterSubsystem;


public class IntakeHingeCommandUp extends Command {
    private final TalonFX intakeHingeTalon;
    private final ShooterSubsystem shooter;
    private double startPosition;
    

    public IntakeHingeCommandUp(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
        intakeHingeTalon = shooter.getHingeMotor();
    }

    @Override
    public void initialize(){
        //intakeHingeTalon.setControl(new PositionDutyCycle(startPosition)); //placeholder
        intakeHingeTalon.set(-0.1);
        shooter.getIntakeMotor().set(0.3);
    }
    @Override
    public void end(boolean interrupted){
        //intakeHingeTalon.setControl(new PositionDutyCycle(startPosition)); //Placeholder 
        intakeHingeTalon.set(0); 
        shooter.getIntakeMotor().set(0);
            
    }
    /*
    @Override
    public boolean isFinished() {
        return false; // keeps running until cancelled
    }*/
}

