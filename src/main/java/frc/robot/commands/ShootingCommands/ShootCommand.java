package frc.robot.commands.ShootingCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;


public class ShootCommand extends Command {
    private ShooterSubsystem shooter;
    private final Timer timer = new Timer();

    public ShootCommand(ShooterSubsystem shooter){
        this.shooter = shooter;
        addRequirements(shooter);
    }
    @Override
    public void initialize(){   
        timer.reset();
        timer.start();
        shooter.getMasterMotor().set(-1);
    }
    @Override
    public void execute() {
        
        if (timer.get() >= 0.75) {
            double t = timer.get();

            // Pulsating speed between 0 and 0.5
            double pulsate = 0.6 + 0.1 * Math.cos(t * 6); 

            shooter.getRollerMotor().set(pulsate);
    }
}

        /*double targetPosition = startPosition + angle;
        shooter.getPitchMotor().setControl(
            new PositionDutyCycle((((targetPosition ))))
        );*/
    
    
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

}
