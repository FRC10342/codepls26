package frc.robot.commands.AutoCommands;

/*import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PitchSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.PositionDutyCycle;


public class ShootAutoCommand extends Command {
    private ShooterSubsystem shooter;
    private PitchSubsystem pitch;
    private final Timer timer = new Timer();
    private double startPosition;

    public ShootAutoCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }
    @Override
    public void initialize(){
        startPosition = pitch.getPitchMotor().getPosition().getValueAsDouble();
        timer.reset();
        timer.start();
        shooter.getMasterMotor().set( -1);
        pitch.getPitchMotor().setControl(new PositionDutyCycle(0.05)); //0.5 is a placeholder, change for the angle required to shoot
    
    }
    @Override
    public void execute() {
        if (timer.get() >= 0.75) {
            double t = timer.get();
            // Pulsating speed between 0 and 0.5
            double pulsate = 0.25 + 0.25 * Math.cos(t * 4); 
            shooter.getRollerMotor().set(pulsate);
        }
}
    
    
    @Override
    public void end(boolean interrupted){
        shooter.getMasterMotor().set(0);
        shooter.getRollerMotor().set(0);
        timer.reset();
    }
    @Override
    public boolean isFinished() {
        return false; // keeps running until cancelled
    }

}*/