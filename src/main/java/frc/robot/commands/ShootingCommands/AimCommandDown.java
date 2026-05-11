package frc.robot.commands.ShootingCommands;

import frc.robot.subsystems.PitchSubsystem;



import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionDutyCycle;

import frc.robot.subsystems.ShooterSubsystem;


public class AimCommandDown extends Command {
    private final TalonFX pitchmotor;
    private final PitchSubsystem pitch;


    public AimCommandDown(PitchSubsystem pitch) {
        this.pitch = pitch;
        addRequirements(pitch);
        pitchmotor = pitch.getPitchMotor();
    
    }

    @Override
    public void execute(){
            pitchmotor.set(-0.05);
            
    }
    @Override
    public void end(boolean interrupted){ 
        pitchmotor.set(0);
        pitchmotor.setNeutralMode(NeutralModeValue.Brake);
        
    }
    /*
    @Override
    public boolean isFinished() {
        return false; // keeps running until cancelled
    }*/
}

    



