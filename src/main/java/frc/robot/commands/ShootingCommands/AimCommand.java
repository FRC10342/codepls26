package frc.robot.commands.ShootingCommands;

import java.util.OptionalDouble;
import java.util.Set;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterPitchMath;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PitchSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Velocity;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

public class AimCommand extends Command{
private final PitchSubsystem pitch;
private final CommandSwerveDrivetrain drivetrain;
private final VisionSubsystem vision;
private double startPosition;
double gravity = 9.81;
double ballvelocity = 17;

public AimCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision, PitchSubsystem pitch){
    this.drivetrain = drivetrain;
    
    this.vision = vision;
    this.pitch = pitch;
    addRequirements(pitch);
    }
    @Override
    public void initialize(){
        //startPosition = 0.005;
    }
    @Override 
    public void execute(){

    pitch.getPitchMotor().set(0.05);
    

    /*double shooterHeightMeters = 0.63; 
    double targetHeightMeters = 1.81;
    
    
    
    Pose2d robotPose = drivetrain.getState().Pose;
    Pose2d targetPose = Constants.FieldConstants.getHubCenter();
    Pose3d targetPose3d = new Pose3d(
        targetPose.getX(),
        targetPose.getY(), 
        targetHeightMeters, 
        new Rotation3d(0, 0, robotPose.getRotation().getRadians())
        );
    Pose3d robotPose3d = new Pose3d(
        robotPose.getX(), 
        robotPose.getY(), 
        shooterHeightMeters, 
        new Rotation3d(0, 0, 0)
        );

    
    OptionalDouble angleOptional = ShooterPitchMath.calculatePitchRadians(
        targetPose,
        robotPose, 
        targetPose3d, 
        robotPose3d,
        ballvelocity);
        
        
          
        
    if (angleOptional.isPresent()) {
        double angleRadians = angleOptional.getAsDouble();
        double targetRotations = startPosition + (angleRadians/((2*Math.PI) * ( 11 )));
        targetRotations = MathUtil.clamp(targetRotations, 0, 1.3);
        pitch.getPitchMotor().setControl(
            new PositionDutyCycle(0.1)//targetRotations)*/
        
    
        
    
    }
    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        //pitch.getPitchMotor().setControl(new PositionDutyCycle(startPosition));
        pitch.getPitchMotor().set(0);
        pitch.getPitchMotor().setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public Set<edu.wpi.first.wpilibj2.command.Subsystem> getRequirements() {
        return Set.of(pitch);
    }
    
}