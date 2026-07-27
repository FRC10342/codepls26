package frc.robot.commands.ShootingCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ZoneBasedTargeting;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class AlignCommand extends Command {

    private final VisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final ZoneBasedTargeting zonesystem;
    
    private final PIDController distanceController = new PIDController(1.5, 0.0, 0.0);
    private final PIDController rotationController = new PIDController(3, 0, 0);
    private static final double MAX_SHOOT_DISTANCE = 50.0;  // meters !!PLACEHOLDER!!
    private static final double DISTANCE_TOLERANCE = 0.1;  // meters
    private static final double ROTATION_TOLERANCE = Math.toRadians(3);

    public AlignCommand(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain, ZoneBasedTargeting zonesystem){
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.zonesystem = zonesystem;
        /*make.robot.drive.good;
        make.robot.shoot.good;
        make.robot.win.worlds.good;*/
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(ROTATION_TOLERANCE);
        distanceController.setTolerance(DISTANCE_TOLERANCE);
        addRequirements(drivetrain, zonesystem, vision);
    }
    @Override
        public void initialize(){
            rotationController.reset();
            distanceController.reset();
        }
    @Override
        public void execute() {
            //find values
            Pose2d robotPose = drivetrain.getState().Pose;
            ZoneBasedTargeting.Zone zone = zonesystem.getCurrentZone(robotPose);
            Pose2d targetPose = Constants.FieldConstants.getHubCenter();
            double allianceWallAngle;
            //Align to hub if in our zone
            if (zone == ZoneBasedTargeting.Zone.OUR_ZONE){
                Translation2d robotToTag = targetPose.getTranslation().minus(robotPose.getTranslation());
                double distance = robotToTag.getNorm();
                Rotation2d targetAngle = robotToTag.getAngle(); // angle robot should face NOT CURRENT ANGLE

                
                double rotationSpeed = rotationController.calculate(
                robotPose.getRotation().getRadians(),
                targetAngle.getRadians()
                );

                if (rotationController.atSetpoint()) {
                rotationSpeed = 0.0;
                }

                double forwardSpeed = 0.0;

                if (distance > MAX_SHOOT_DISTANCE) {
                // Drive toward the hub until just inside shooting range
                forwardSpeed = distanceController.calculate(distance, MAX_SHOOT_DISTANCE - 0.05);

                // Optional: prevent moving forward if robot is badly misaligned
                if (Math.abs(rotationController.getPositionError()) > 0.35) {
                    forwardSpeed = 0;
                }
                }

                rotationSpeed = MathUtil.clamp(rotationSpeed, -0.5, 0.5);
                forwardSpeed = MathUtil.clamp(forwardSpeed, -2.0, 2.0);

                // feed speeds to drivetrain
                //drivetrain.setAlignRequest(forwardSpeed, 0.0, rotationSpeed);
                 drivetrain.setControl(
                    new SwerveRequest.FieldCentric()
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(rotationSpeed)
                );
                }
                else
                {//should align back to alliance wall to shoot balls back into our zone
                    if (DriverStation.getAlliance().isPresent() &&
                    DriverStation.getAlliance().get() == Alliance.Blue) {
                    allianceWallAngle = Math.PI;   // face Blue wall
                    } else {
                    allianceWallAngle = 0.0;       // face Red wall
                    }
                    double rotationSpeed = rotationController.calculate(
                    robotPose.getRotation().getRadians(),
                    allianceWallAngle
                    );
                    if (rotationController.atSetpoint()) {
                    rotationSpeed = 0.0;
                    }   
                     drivetrain.setControl(
                        new SwerveRequest.FieldCentric()
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(rotationSpeed)
                    );
                    //drivetrain.setAlignRequest(0.0, 0.0, rotationSpeed);
                }
            }
            
        @Override
        public void end(boolean interrupted) {
            // stop drivetrain when command ends
            //drivetrain.setAlignRequest(0, 0, 0);
            drivetrain.setControl(
                        new SwerveRequest.FieldCentric()
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0)
                    );
        }
        @Override
        public boolean isFinished() {
        return false; // keeps running until cancelled (e.g. trigger released)
        }

        /** True once the drivetrain is facing the current target heading within tolerance. */
        public boolean isAligned() {
        return rotationController.atSetpoint();
        }
    }


    

