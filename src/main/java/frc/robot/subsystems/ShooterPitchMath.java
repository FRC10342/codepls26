package frc.robot.subsystems;

import java.util.OptionalDouble;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public final class ShooterPitchMath {
private static final double gravity = 9.81;
private ShooterPitchMath() {}
public double distanceAwayFromGoal;



public static OptionalDouble calculatePitchRadians(
Pose2d targetPose,
Pose2d robotPose,
Pose3d targetPose3d,
Pose3d robotPose3d,
double velocity




) {
    
    double shooterHeightMeters = 0.63; 
    double targetHeightMeters = 1.81;
    double distanceAwayFromGoal =
        robotPose.getTranslation().getDistance(targetPose.getTranslation());

    double heightDifferenceMeters = targetHeightMeters - shooterHeightMeters;
    double velocity2 = velocity * velocity;

    double discriminat = velocity2 * velocity2 - gravity * ( gravity * distanceAwayFromGoal * distanceAwayFromGoal + (2 * heightDifferenceMeters * velocity2));
    if (discriminat < 0){
        return OptionalDouble.empty();
    }

    double sqrt = Math.sqrt(discriminat);

    double angle = Math.atan((velocity2 - sqrt) / (gravity * distanceAwayFromGoal));




    return OptionalDouble.of(angle);
}
}
