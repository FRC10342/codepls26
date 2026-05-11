package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ShooterSubsystem;


public class IntakeCommand extends Command {

    private ShooterSubsystem shooter;
    //private final Timer time = new Timer();
    public static Command IntakeCommand;
    private final TalonFX IntakeTalon;
    //private boolean On = false;
    //private final TalonFX IntakeHingeTalons;

    public IntakeCommand(ShooterSubsystem shooter){
    this.shooter = shooter;
    addRequirements(shooter);
    this.IntakeTalon = shooter.getIntakeMotor(); //Device ID placeholder
    //this.IntakeHingeTalons = new TalonFX(11); //Device ID placeholder
    }
    @Override
    public void initialize(){
        //IntakeHingeTalons.setPosition(90); //Placeholder
        //time.delay(0.25); //Placeholder
        IntakeTalon.set(0.45);
    }
    @Override
    public void end(boolean interrupted){
        IntakeTalon.set(0);
        //IntakeHingeTalons.setPosition(0);
        
    }
    /* 
    @Override
    public boolean isFinished() {        
        return false; // keeps running until cancelled
    }*/
} 

