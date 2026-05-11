package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PitchSubsystem extends SubsystemBase{
private final TalonFX pitchMotor = new TalonFX(16, "CANsucks");
public PitchSubsystem(){
    // Configure PID for pitch
        TalonFXConfiguration pitchconfig = new TalonFXConfiguration();
        pitchconfig.Slot0.kP = 2;
        pitchconfig.Slot0.kI = 1.3;
        pitchconfig.Slot0.kD = 0.05;
        pitchconfig.Slot0.kV = 0;
        pitchconfig.MotorOutput.PeakForwardDutyCycle = 0.1;
        pitchconfig.MotorOutput.PeakReverseDutyCycle = -0.05;
        pitchconfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        pitchconfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1.3;
        pitchconfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        pitchconfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        
        pitchMotor.getConfigurator().apply(pitchconfig);
}
public TalonFX getPitchMotor() {
        return pitchMotor;
    }

}