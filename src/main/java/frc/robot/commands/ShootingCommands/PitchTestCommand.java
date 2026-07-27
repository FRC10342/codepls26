package frc.robot.commands.ShootingCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PitchSubsystem;

import com.ctre.phoenix6.controls.PositionDutyCycle;


public class PitchTestCommand extends Command {
    private double angle;
    private double startPosition;
    private PitchSubsystem pitch;
    public PitchTestCommand(PitchSubsystem pitch, double angle){
            this.angle = angle;
            this.pitch = pitch;
            addRequirements(pitch);
    }
    @Override
    public void initialize(){
        startPosition = 0;
    }

    @Override
    public void execute(){
        pitch.getPitchMotor().setControl(
            new PositionDutyCycle(angle)
        );
    }
    @Override
    public void end(boolean interrupted){
        pitch.getPitchMotor().setControl(
            new PositionDutyCycle(startPosition)
        );

    }
    @Override
    public boolean isFinished() {
        return false; // keeps running until cancelled
    }
}

