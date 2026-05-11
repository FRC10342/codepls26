package frc.robot.commands.ShootingCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ZoneBasedTargeting;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;

public class AlignTest extends Command {
    private VisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final PIDController rotationController = new PIDController(3, 0, 0);
    private static final double ROTATION_TOLERANCE = Math.toRadians(3);

    public AlignTest(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain){
        this.vision = vision;
        this.drivetrain = drivetrain;
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(ROTATION_TOLERANCE);
        addRequirements(drivetrain);
    }
    @Override
    public void initialize() {
        rotationController.reset();
    }
    @Override
        public void execute() {
            //find values
            Pose2d robotPose = drivetrain.getState().Pose;
            //System.out.println("align has robotPose:"+ robotPose);
            Optional <Pose2d> optionaltagPose = vision.getRobotToTagPose();
            if (optionaltagPose.isEmpty()) {
                //System.out.println("No tag detected");
                return;
            }
            if (optionaltagPose.isPresent()){
                Pose2d tagPose = optionaltagPose.get();
                //System.out.println("align request found tag");
                Translation2d robotToTag = tagPose.getTranslation();
                Rotation2d targetAngle = robotToTag.getAngle(); // angle robot should face NOT CURRENT ANGLE
                //System.out.println("angle:" + targetAngle);
                double rotationSpeed = rotationController.calculate(
                robotPose.getRotation().getRadians(),
                targetAngle.getRadians()
                );

                if (rotationController.atSetpoint()) {
                rotationSpeed = 0.0;
                }
                //System.out.println("rotspeed:" + rotationSpeed);
                rotationSpeed = MathUtil.clamp(rotationSpeed, -2.5, 2.5);
                //feed speeds to drivetrain
                drivetrain.setControl(
                new SwerveRequest.FieldCentric()
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(rotationSpeed)
            );
                //System.out.println("applying request");
                }
        }
        @Override
            public void end(boolean interrupted) {
            drivetrain.setControl(
                new SwerveRequest.FieldCentric()
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
            );
            //System.out.println("stopping align");
        }
        
        @Override
        public boolean isFinished() {
            return false; //rotationController.atSetpoint();
        }
    }
 
