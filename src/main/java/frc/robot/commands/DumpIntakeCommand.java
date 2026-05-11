package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.subsystems.ShooterSubsystem;


public class DumpIntakeCommand extends Command {

    private ShooterSubsystem shooter;
    public static Command IntakeCommand;
    private final TalonFX IntakeTalon;
    private final TalonFX rollerTalon;
    private final TalonFX shooterTalon;
    //private final TalonFX IntakeHingeTalons;

    public DumpIntakeCommand(ShooterSubsystem shooter){
    this.shooter = shooter;
    addRequirements(shooter);
    this.IntakeTalon = shooter.getIntakeMotor(); //Device ID placeholder
    //this.IntakeHingeTalons = new TalonFX(11); //Device ID placeholder
    this.rollerTalon = shooter.getRollerMotor();
    this.shooterTalon = shooter.getMasterMotor();
    }
    @Override
    public void initialize(){
        //IntakeHingeTalons.setPosition(90); //Placeholder
        //time.delay(0.25); //Placeholder
        IntakeTalon.set(-0.5);
        rollerTalon.set(0.5);
        shooterTalon.set(0.2);
        }

    
    @Override
    public void end(boolean interrupted){      
        IntakeTalon.set(0);
        rollerTalon.set(0);
        shooterTalon.set(0);
        //IntakeHingeTalons.setPosition(0);
        
    }
    /* 
    @Override
    public boolean isFinished() {        
        return false; // keeps running until cancelled
    }*/
} 

