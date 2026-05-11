package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.controls.PositionDutyCycle;

import frc.robot.subsystems.ShooterSubsystem;


public class IntakeAutoCommand extends Command {
    private final TalonFX intakeHingeTalon;
    private final TalonFX intakeRollerTalon;
    private final ShooterSubsystem shooter;
    private double startPosition;
    public static Command IntakeCommand;

    public IntakeAutoCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
        intakeHingeTalon = shooter.getHingeMotor();
        intakeRollerTalon = shooter.getIntakeMotor();
    }

    @Override
    public void initialize(){
        //intakeHingeTalon.setControl(new PositionDutyCycle(startPosition)); //placeholder
        intakeHingeTalon.set(0.2);
        intakeRollerTalon.set(0.45);
    }
    @Override
    public void end(boolean interrupted){
        //intakeHingeTalon.setControl(new PositionDutyCycle(startPosition)); //Placeholder 
        intakeHingeTalon.set(-0.2); 
        intakeRollerTalon.set(0);   
    }

    @Override
    public boolean isFinished() {
        return false; // keeps running until cancelled
    }
}

