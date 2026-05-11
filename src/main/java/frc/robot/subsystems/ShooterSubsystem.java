package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX masterMotor = new TalonFX(23, "CANsucks");
    private final TalonFX followerMotor = new TalonFX(24, "CANsucks");
    private final TalonFX rollerMotor = new TalonFX(25, "CANsucks");
    private final TalonFX intakeMotor = new TalonFX(22, "CANsucks");
    private final TalonFX climbMotor = new TalonFX(17, "CANsucks");
    private final TalonFX hingeMotor = new TalonFX(11, "CANsucks");

    public ShooterSubsystem(){
        followerMotor.setControl(new Follower(23, MotorAlignmentValue.Opposed));
        
        TalonFXConfiguration climbconfig = new TalonFXConfiguration();
        climbconfig.Slot0.kP = 0;
        climbconfig.Slot0.kI = 0;
        climbconfig.Slot0.kD = 0;
        climbconfig.Slot0.kV = 0;
        climbconfig.MotorOutput.PeakForwardDutyCycle = 1;
        climbconfig.MotorOutput.PeakReverseDutyCycle = -1;
        //climbconfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ;
        //climbconfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
        climbMotor.getConfigurator().apply(climbconfig);
    }
    public TalonFX getMasterMotor() {
        return masterMotor;
    }
    
    public TalonFX getFollowerMotor() {
        return followerMotor;
    }

    public TalonFX getRollerMotor() {
        return rollerMotor;
    }

    public TalonFX getIntakeMotor() {
        return intakeMotor;
    }

    public TalonFX getClimbMotor() {
        return climbMotor;
    }

    public TalonFX getHingeMotor() {
        return hingeMotor;
    }

    public void stopAll() {
        masterMotor.set(0);
        rollerMotor.set(0);
        followerMotor.set(0);
        hingeMotor.set(0);
        climbMotor.set(0);
        intakeMotor.set(0);
    }
}

