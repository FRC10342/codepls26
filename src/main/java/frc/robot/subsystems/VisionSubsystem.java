package frc.robot.subsystems;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

//Vision imports
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.opencv.photo.Photo;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import org.photonvision.targeting.PhotonPipelineResult;

public class VisionSubsystem extends SubsystemBase {
    private Optional<Integer> latestTagID = Optional.empty();
    private Optional<Transform3d> latestCameraToTag = Optional.empty();
    private Optional<Transform3d> latestRobotToTag = Optional.empty();

    private static class VisionCamera {
        public final PhotonCamera camera;
        public final PhotonPoseEstimator estimator;

        public VisionCamera(
                String name,
                Transform3d robotToCam,
                AprilTagFieldLayout layout) {

                camera = new PhotonCamera(name);

                estimator = new PhotonPoseEstimator(
                    layout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    robotToCam
            );
            estimator.setMultiTagFallbackStrategy(
                    PoseStrategy.LOWEST_AMBIGUITY
            );
            }

    }
    private final List<VisionCamera> cameras = new ArrayList<>();
    
    private Optional<EstimatedRobotPose> latestEstimate = Optional.empty();
    private final CommandSwerveDrivetrain drivetrain;
    
    private final AprilTagFieldLayout tagLayout;
    public VisionSubsystem(CommandSwerveDrivetrain drivetrain){
        this.drivetrain = drivetrain;
        //load AprilTag field layout
        try {
            tagLayout = AprilTagFieldLayout.loadFromResource(
            AprilTagFields.k2026RebuiltAndymark.m_resourceFile
            );
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        //  Define cameratorobot transforms (placeholder)
            Transform3d robotTofrtrightCam = new Transform3d(
                    new Translation3d(0.355, 0.0325, 0.225),
                    new Rotation3d(0, 0.48, 0.5672)
            );
            Transform3d robotTofrtleftCam = new Transform3d(
                    new Translation3d(0.355,-0.0325,0.225),
                    new Rotation3d(0,0.48,-0.5672)
            );
        cameras.add(new VisionCamera("frtrightCamera", robotTofrtrightCam, tagLayout));
        cameras.add(new VisionCamera("frtleftCamera", robotTofrtleftCam, tagLayout));
    }
    /*private boolean isScoringTag(int id) {
    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

    if (alliance == DriverStation.Alliance.Blue) {
        return id == 7;   // Blue scoring tag ID !!PLACEHOLDERS!!
    } else {
        return id == 4;   // Red scoring tag ID !!PLACEHOLDER!!
    }
    }*/

    private void processCamera(
    VisionCamera cam,
    Pose3d referencePose3d,
    Pose2d currentPose) {
    PhotonPipelineResult result = cam.camera.getLatestResult();
    if (!result.hasTargets()) {
        //System.out.println("No targets detected");
        return;
    } 
    //System.out.println("Target detected: " + result.getBestTarget().getFiducialId());

    var bestTarget = result.getBestTarget();
    if (bestTarget.getPoseAmbiguity() > 0.2) return;
    //System.out.println("CamToTag X: " + bestTarget.getBestCameraToTarget().getX());
    //System.out.println("CamToTag Y: " + bestTarget.getBestCameraToTarget().getY());
    //System.out.println("Yaw: " + bestTarget.getYaw());

    latestTagID = Optional.of(bestTarget.getFiducialId());

    //System.out.println("targetid:" + latestTagID);

    latestCameraToTag = Optional.of(bestTarget.getBestCameraToTarget());

    if(latestCameraToTag.isPresent()){
        Transform3d CameraToTag = latestCameraToTag.get();
        latestRobotToTag = Optional.of(cam.estimator.getRobotToCameraTransform().plus(CameraToTag));
    }
   

    //System.out.println("found robot to tag translation");

    Optional<EstimatedRobotPose> estimate = cam.estimator.estimateCoprocMultiTagPose(result);
    System.out.println("CoprocMultiTagPose: " + estimate.isPresent());

    if (estimate.isEmpty()) {
        estimate = cam.estimator.estimateClosestToReferencePose(
            result, referencePose3d);
        System.out.println("ClosestToReferencePose: " + estimate.isPresent());
    }

    if (estimate.isEmpty()) {
        estimate = cam.estimator.estimateLowestAmbiguityPose(result);
        System.out.println("LowestAmbiguityPose: " + estimate.isPresent());
    }

    if (estimate.isEmpty()){ 
    System.out.println("no pose");
    return;
    }
    else{
    
    System.out.println("we made it");

    Pose2d visionPose = estimate.get().estimatedPose.toPose2d();
    System.out.println("estimate found");
    //double timestamp = estimate.get().timestampSeconds;
    double timestamp = estimate.get().timestampSeconds;

    double jump = visionPose.getTranslation().getDistance(currentPose.getTranslation());

    if (jump > 1.0) return;
    // Feed vision pose into drivetrain Kalman filter
    drivetrain.addVisionMeasurement(visionPose, timestamp);
    System.out.println("Adding vision measurement: " + visionPose);
    }
    
    latestEstimate = estimate; // optional: store last valid

    
    }


    @Override
    // Get the latest estimated pose from PhotonVision through 3 different methods for safety
    public void periodic() {
    //get a recent robot pose from drivetrain
    Pose2d currentPose = drivetrain.getState().Pose;
    //camera height in meters add later
    double cameraHeightMeters = 0.225;
    //create Pose3d from sample Pose2d
    Pose3d referencePose3d = new Pose3d(
    currentPose.getX(),
    currentPose.getY(),
    cameraHeightMeters,          // z (meters) of camera on robot
    new Rotation3d(0, 0, currentPose.getRotation().getRadians()));


    for (VisionCamera cam : cameras) {
            processCamera(cam, referencePose3d, currentPose);
        }
    }
    public Optional<Pose2d> getLatestVisionPose() {
    return latestEstimate.map(
        est -> est.estimatedPose.toPose2d()
    );
    }
   
    public Optional<Transform3d> getLatestRobotToTag() {
        return latestRobotToTag;
        
    }
    public Optional<Pose2d> getRobotToTagPose() {
        return latestRobotToTag.map(transform -> 
            new Pose2d(transform.getX(), transform.getY(), transform.getRotation().toRotation2d())
        );
    }
}

// High quality vision = True
// With this command i say that this bot will hit a frontflip and completely dump hopper in 2.3 seconds