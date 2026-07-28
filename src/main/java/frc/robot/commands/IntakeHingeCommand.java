package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.controls.PositionDutyCycle;

import frc.robot.subsystems.ShooterSubsystem;

public class IntakeHingeCommand extends Command {
    private final TalonFX intakeHingeTalon;
    private final ShooterSubsystem shooter;
    private double startPosition;
    

    public IntakeHingeCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
        intakeHingeTalon = shooter.getHingeMotor();
    
    }

    @Override
    public void initialize(){
        //intakeHingeTalon.setControl(new PositionDutyCycle(startPosition)); //placeholder
        // NOTE: sign is an unverified guess (opposite of IntakeHingeCommandUp's -0.1 "up" direction) -
        // confirm which way this actually drives the hinge on real hardware, at low power, before trusting it.
        intakeHingeTalon.set(0.1);
        shooter.getIntakeMotor().set(0.45);

    }
    @Override
    public void end(boolean interrupted){
        //intakeHingeTalon.setControl(new PositionDutyCycle(startPosition)); //Placeholder
        intakeHingeTalon.set(0);
        shooter.getIntakeMotor().set(0); // was left running forever: initialize() drives this motor but end() never stopped it

    startPosition = intakeHingeTalon.getPosition().getValueAsDouble();

    }
    /*
    @Override
    public boolean isFinished() {
        return false; // keeps running until cancelled
    }*/
}

